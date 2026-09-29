"""Fetch CC0 PBR sources serially, and prepare small reproducible colour previews.

Previews are not native scan textures. Full-resolution baking must replay the
saved colour transform in strips/tiles; no whole-tunnel image is allocated here.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time
import zipfile

import numpy as np
from PIL import Image, ImageDraw

from .stage_b_scene import digest, load_spec, peak_rss_bytes


POLY = ('plastered_wall_04', 'plastered_wall_03')
CHANNELS = {'diffuse':'diff', 'roughness':'rough', 'normal_gl':'nor_gl'}


def sources_match_spec(sources, spec):
    # Crack/layout updates do not alter the identity of already downloaded PBR maps.
    materials=sources.get('materials',[])
    return ([m['id'] for m in materials]==spec['materials']['sources'] and
            all(m['source_width_m']==spec['materials']['source_width_m'][m['id']] for m in materials))


def download(url, target, limit):
    target = Path(target)
    if target.exists():
        raise ValueError('refuse to overwrite unverified download: '+str(target))
    part = target.with_name(target.name+'.part')
    # curl streams to disk, limits advertised/actual transfer size, and fails HTTP errors.
    subprocess.run(['curl','--fail','--silent','--show-error','--location','--retry','2',
                    '--connect-timeout','15','--max-time','600','--max-filesize',str(limit),
                    '--user-agent','Mozilla/5.0',url,'-o',str(part)], check=True)
    if part.stat().st_size == 0 or part.stat().st_size > limit:
        raise ValueError('download outside size budget')
    part.rename(target)


def image_info(path, memory_budget):
    with Image.open(path) as image:
        w, h = image.size
        # Conservative transient estimate for decoding and a later bounded colour operation.
        estimate = w*h*64 + (2 << 20)
        if estimate > memory_budget:
            raise ValueError(f'source exceeds preparation memory estimate: {estimate} > {memory_budget}; '
                             'use a lower resolution or a streaming decoder')
        image.verify()
    return dict(file=path.name, resolution=[w,h], sha256=digest(path), bytes=path.stat().st_size)


def fetch(output, spec_path, resolution='2k'):
    spec = load_spec(spec_path)
    side = {'1k':1024,'2k':2048,'4k':4096}[resolution]
    # Refuse before downloading a source that the current nonstreaming decoder cannot process.
    if side*side*64+(2 << 20) > spec['resources']['asset_working_set_bytes']:
        raise ValueError('resolution exceeds decoder budget; native final sources need streaming decoding')
    output = Path(output).resolve()
    manifest_path = output/'sources.json'
    if manifest_path.exists():
        old = json.loads(manifest_path.read_text())
        if old['resolution'] != resolution or not sources_match_spec(old,spec):
            raise ValueError('existing material cache belongs to another recipe')
        for material in old['materials']:
            for info in material['channels'].values():
                if digest(output/material['id']/info['file']) != info['sha256']:
                    raise ValueError('material cache hash mismatch')
        print(json.dumps(dict(status='verified_existing', output=str(output))))
        return old
    if output.exists():
        raise ValueError('incomplete material directory already exists: '+str(output))
    output.mkdir(parents=True)
    start = time.monotonic()
    limit, memory = (spec['resources'][k] for k in ('max_download_bytes','asset_working_set_bytes'))
    materials = []
    for asset in spec['materials']['sources']:
        folder = output/asset
        folder.mkdir()
        item = dict(id=asset, license='CC0', source_width_m=spec['materials']['source_width_m'][asset],
                    scale_basis='engineering_assumption' if asset=='Concrete030' else 'official_asset_page',
                    source_url=('https://ambientcg.com/view?id='+asset if asset=='Concrete030' else 'https://polyhaven.com/a/'+asset), channels={})
        if asset in POLY:
            for channel, suffix in CHANNELS.items():
                name = f'{asset}_{suffix}_{resolution}.png'
                url = f'https://dl.polyhaven.org/file/ph-assets/Textures/png/{resolution}/{asset}/{name}'
                download(url, folder/name, limit)
                item['channels'][channel] = dict(**image_info(folder/name,memory), download_url=url)
        elif asset == 'Concrete030':
            name = f'{asset}_{resolution.upper()}-PNG.zip'
            url = 'https://ambientcg.com/get?file='+name
            archive = folder/name
            download(url, archive, limit)
            item['archive'] = dict(file=name,sha256=digest(archive),download_url=url)
            with zipfile.ZipFile(archive) as zipped:
                for channel, suffix in (('diffuse','Color'),('roughness','Roughness'),('normal_gl','NormalGL')):
                    member = f'{asset}_{resolution.upper()}-PNG_{suffix}.png'
                    entry = zipped.getinfo(member)
                    if entry.file_size > memory:
                        raise ValueError('uncompressed source exceeds extraction budget')
                    with zipped.open(member) as src, (folder/member).open('wb') as dest:
                        shutil.copyfileobj(src, dest, length=1 << 20)
                    item['channels'][channel] = image_info(folder/member,memory)
        else:
            raise ValueError('unsupported material source: '+asset)
        for info in item['channels'].values():
            if info['resolution'] != [side,side]:
                raise ValueError('source dimensions differ from requested resolution')
        materials.append(item)
    manifest = dict(schema='ssb.material_sources.v1',resolution=resolution,spec_sha256=digest(spec_path),
                    materials=materials,download_seconds=time.monotonic()-start,
                    preparation_peak_rss_bytes=peak_rss_bytes(), purpose='preview_and_pipeline_development',
                    note='Low-resolution source package; not a claim of 0.2 mm background detail.')
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(status='downloaded',output=str(output),seconds=manifest['download_seconds'],peak_rss_bytes=manifest['preparation_peak_rss_bytes'])))
    return manifest


def srgb_to_linear(a):
    a = np.asarray(a,dtype=np.float32)
    return np.where(a<=.04045,a/12.92,((a+.055)/1.055)**2.4)


def linear_to_srgb(a):
    a = np.clip(np.asarray(a,dtype=np.float32),0,1)
    return np.where(a<=.0031308,12.92*a,1.055*a**(1/2.4)-.055)


def stats(linear):
    pixels = linear.reshape(-1,3)
    median = np.median(pixels,axis=0)
    low, high = np.quantile(pixels,[.1,.9],axis=0)
    return median, np.maximum(high-low,1e-3)


def colour_transform(source, reference):
    centre, spread = stats(source)
    target, target_spread = stats(reference)
    # Bounded contrast gain prevents a smooth source's sensor/compression noise exploding.
    gain = np.clip(target_spread/spread,.5,2.)
    return dict(gain=gain.tolist(),offset=(target-centre*gain).tolist(),
                source_median=centre.tolist(),target_median=target.tolist(),domain='linear_srgb',
                definition='one global per-source transform; not per-patch equalisation')


def apply_transform(linear, recipe, brightness):
    return np.clip((linear*np.asarray(recipe['gain'])+np.asarray(recipe['offset']))*brightness,0,1)


def preview(sources_path, spec_path, output, side=384):
    spec = load_spec(spec_path)
    limit = spec['resources']['asset_working_set_bytes']
    if side <= 0 or side > 1024 or 12*side*side*64 > limit:
        raise ValueError('preview exceeds preparation budget')
    sources_path = Path(sources_path).resolve()
    sources = json.loads(sources_path.read_text())
    if not sources_match_spec(sources,spec):
        raise ValueError('source recipe changed')
    out = Path(output).resolve()
    if out.exists():
        raise ValueError('preview output already exists')
    out.mkdir(parents=True)
    images = {}
    for item in sources['materials']:
        entry = item['channels']['diffuse']
        path = sources_path.parent/item['id']/entry['file']
        if digest(path) != entry['sha256']:
            raise ValueError('source diffuse hash mismatch')
        image_info(path,limit)
        with Image.open(path) as image:
            small = image.convert('RGB').resize((side,side),Image.Resampling.LANCZOS)
        images[item['id']] = srgb_to_linear(np.asarray(small)/255.)
    ref = images[spec['materials']['reference']]
    transforms = {name:colour_transform(image,ref) for name,image in images.items()}
    factors = spec['materials']['brightness_factors']
    # A comparison chart assembled from computed small previews, not source assets.
    canvas = Image.new('RGB',(len(images)*side,len(factors)*(side+30)),(225,225,225))
    draw = ImageDraw.Draw(canvas)
    for col, (name,linear) in enumerate(images.items()):
        for row, factor in enumerate(factors):
            corrected = apply_transform(linear,transforms[name],factor)
            encoded = np.uint8(np.clip(linear_to_srgb(corrected)*255+.5,0,255))
            small = Image.fromarray(encoded)
            canvas.paste(small,(col*side,row*(side+30)+30))
            draw.text((col*side+8,row*(side+30)+8),f'{name} | linear reflectance x{factor}',fill=(20,20,20))
    canvas.save(out/'colour_comparison.png')
    recipe = dict(schema='ssb.colour_recipe.v1',sources_sha256=digest(sources_path),spec_sha256=digest(spec_path),
                  reference=spec['materials']['reference'],transforms=transforms,brightness_factors=factors,
                  final_brightness_selected=None,preview_side=side,
                  note='Preview selection pending; transformation is an appearance adjustment, not measured reflectance.',
                  preparation_peak_rss_bytes=peak_rss_bytes())
    (out/'colour_recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),peak_rss_bytes=recipe['preparation_peak_rss_bytes'])))
    return recipe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command',required=True)
    get = subparsers.add_parser('fetch')
    get.add_argument('--resolution',choices=['1k','2k','4k'],default='2k')
    get.add_argument('--spec',required=True,type=Path)
    get.add_argument('--output',required=True,type=Path)
    view = subparsers.add_parser('preview')
    view.add_argument('--sources',required=True,type=Path)
    view.add_argument('--spec',required=True,type=Path)
    view.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    if args.command=='fetch':
        fetch(args.output,args.spec,args.resolution)
    else:
        preview(args.sources,args.spec,args.output)


if __name__ == '__main__':
    main()

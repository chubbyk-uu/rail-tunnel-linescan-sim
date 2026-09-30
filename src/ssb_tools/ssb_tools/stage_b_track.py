"""Straight rail track: metric rail section, discrete sleepers and bounded collision geometry."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .stage_b_scene import sub,box,cylinder,Mesh


def friction(collision,mu=1.0):
    surface=sub(collision,'surface');f=sub(surface,'friction');ode=sub(f,'ode')
    sub(ode,'mu',mu);sub(ode,'mu2',mu)
    # DART consumes Coulomb friction; ODE contact stiffness is deliberately not claimed.


def finish(link,metalness=.0,roughness=.8):
    for v in link.findall('visual'):
        mat=v.find('material');p=sub(sub(mat,'pbr'),'metal')
        sub(p,'metalness',metalness);sub(p,'roughness',roughness)
        sub(v,'cast_shadows','true')


def profile_mesh(path,profile,x0,x1):
    limits=dict(max_scene_vertices=10000,max_mesh_triangles=10000)
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(profile,profile[1:]+profile[:1]))
    if area>0: profile=list(reversed(profile))
    mesh=Mesh(path,dict(vertices=0,triangles=0),limits)
    # Profile runs clockwise when looking in +x, giving outward side faces.
    for a,b in zip(profile,profile[1:]+profile[:1]):
        mesh.quad([(x0,*a),(x1,*a),(x1,*b),(x0,*b)])
    centre=tuple(sum(p[i] for p in profile)/len(profile) for i in range(2))
    for a,b in zip(profile,profile[1:]+profile[:1]):
        mesh.quad([(x0,*centre),(x0,*a),(x0,*b),(x0,*centre)])
        mesh.quad([(x1,*centre),(x1,*b),(x1,*a),(x1,*centre)])
    mesh.close()


def make_track(out,config,spec):
    out=Path(out);folder=out/'track';folder.mkdir(exist_ok=True)
    t=spec['track'];x0=config['tunnel']['x_min_m'];x1=config['tunnel']['x_max_m'];length=x1-x0
    model=ET.Element('model',name='track');sub(model,'static','true')
    rails=sub(model,'link',name='rails');width=t['head_width_m'];gauge=t['gauge_m'];head_y=(gauge+width)/2
    # 73 x 38 mm head with a 7.5 mm corner radius; top running plane remains z=0.
    r=.0075
    head=[(-width/2+r,0),(width/2-r,0),(width/2-r+r*.707,-r+r*.707),(width/2,-r),
          (width/2,-.0305),(width/2-r+r*.707,-.0305-r*.707),(width/2-r,-.038),
          (-width/2+r,-.038),(-width/2+r-r*.707,-.0305-r*.707),(-width/2,-.0305),
          (-width/2,-r),(-width/2+r-r*.707,-r+r*.707)]
    foot=[(-.075,-.176),(-.075,-.168),(-.020,-.153),(.020,-.153),(.075,-.168),(.075,-.176)]
    web=[(-.00825,-.038),(.00825,-.038),(.00825,-.153),(-.00825,-.153)]
    for name,profile in [('head',head),('web',web),('foot',foot)]:profile_mesh(folder/(name+'.obj'),profile,x0,x1)
    for side,sign in [('left',1),('right',-1)]:
        y=sign*head_y
        for name,color,metalness,rough in [('head','0.48 0.51 0.54 1',.8,.3),('web','0.23 0.16 0.115 1',.4,.75),('foot','0.20 0.135 0.10 1',.4,.75)]:
            v=sub(rails,'visual',name=side+'_'+name);sub(v,'pose',f'0 {y} 0 0 0 0')
            sub(sub(sub(v,'geometry'),'mesh'),'uri',str((folder/(name+'.obj')).resolve()))
            mat=sub(v,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
            p=sub(sub(mat,'pbr'),'metal');sub(p,'metalness',metalness);sub(p,'roughness',rough);sub(v,'cast_shadows','true')
        col=sub(rails,'collision',name=side+'_head');sub(col,'pose',f'{(x0+x1)/2} {y} -.019 0 0 0')
        sub(sub(sub(col,'geometry'),'box'),'size',f'{length} {width} .038');friction(col,1.0)
    sleepers=sub(model,'link',name='sleepers')
    spacing=t.get('sleeper_spacing_m',.6)
    for i in range(math.ceil((x1-x0)/spacing)):
        x=x0+(i+.5)*spacing
        if x+.12>x1:break
        # Dark weathered concrete; slight per-sleeper tint, no texture allocation.
        shade=.20+.012*math.sin(i*2.39996323)
        color=f'{shade*.98:.5f} {shade:.5f} {shade*.96:.5f} 1'
        box(sleepers,f'sleeper_{i}',f'{x} 0 -.276 0 0 0','.24 2.1 .16',color)
        for sign in [-1,1]:
            y=sign*head_y;tag=f'{i}_{sign}'
            box(sleepers,'rubber_'+tag,f'{x} {y} -.192 0 0 0','.25 .20 .008','0.065 0.065 0.060 1')
            box(sleepers,'plate_'+tag,f'{x} {y} -.182 0 0 0','.24 .19 .012','0.21 0.22 0.23 1')
            for d in [-.086,.086]:
                cylinder(sleepers,'bolt_'+tag+str(d),f'{x} {y+d} -.168 0 0 0',.010,.018,'0.26 0.25 0.23 1')
                box(sleepers,'clip_'+tag+str(d),f'{x} {y+d*.84} -.163 0 0 0','.055 .033 .009','0.18 0.14 0.10 1')
    finish(sleepers)
    bed=sub(model,'link',name='bed')
    box(bed,'concrete_bed',f'{(x0+x1)/2} 0 -.466 0 0 0',f'{length} {t["bed_width_m"]} .22','0.27 0.28 0.27 1',True)
    box(bed,'foundation',f'{(x0+x1)/2} 0 -.6455 0 0 0',f'{length} {t["bed_width_m"]} .179','0.25 0.26 0.25 1')
    finish(bed)
    return model

# 设计参考资料

以下为此前设计采用的来源，不是本次重新检索或验证。专利 PDF 未纳入 Git，首次克隆不会包含这些本地文件。

## 15. 参考资料

专利链接指向 Google Patents 公开页面；作者本机保存的 PDF 副本（仓库根目录，已在 `.gitignore` 中）不随仓库分发。

- [CN111185894A：一种隧道轨道检测机器人](https://patents.google.com/patent/CN111185894A/zh)：机构与采集装置参考。未据此认定扫描头与底盘存在物理联动。
- [CN112270672A：基于线阵相机的隧道管片表观图像处理方法](https://patents.google.com/patent/CN112270672A/zh)：扫描几何和图像处理参考。原示例中 4000 像素、70 kHz 等参数不覆盖本次确定的 4096 像素及 ×128÷15 配置。
- 4WIDS 当前状态（相邻私有项目 `4WIDS_agv/docs/CURRENT_STATUS.md`，未公开）：已有工程状态与性能背景。
- 4WIDS 拼接设计（相邻私有项目 `4WIDS_agv/docs/STITCHING_DESIGN.md`，未公开）：连续条带、可测输入、全局修正和质量评估的设计参考；具体几何需改为本项目的圆柱螺旋扫描。
- 轨道尺寸来源：[60 kg/m 钢轨型式尺寸](https://baike.lcgt.cn/entry.html?id=1760)（引 TB/T 2341）；[地铁板式道床轨道结构高度 735 mm 分解](https://www.guimei8.com/10839.html)；[北京大兴机场线道床结构高度实例](https://www.guimei8.com/17214.html)。均为二手资料，实际线路以设计文件为准。
- 管片分块：李围，何川．地铁区间盾构隧道管片衬砌设计分块的探讨．隧道建设，2003，23(6)：1–2，5。
- 管片构造：[天津市快速轨道交通盾构隧道设计规程 DB/T 29-272-2019](https://zfcxjs.tj.gov.cn/sylm/gabsycs/xxbzgfgh/202010/W020201029539664858051.pdf) 第 9 章；国家标准 GB/T 22082-2024《预制混凝土衬砌管片》全文未取得。
- 轨道不平顺：Wang F. et al., [The Research of Irregularity Power Spectral Density of Beijing Subway](https://link.springer.com/article/10.1007/s40864-015-0023-8), Urban Rail Transit 2015, 1(3):159–163（表 2、图 1）；GB/T 50299-2018《地下铁道工程施工质量验收标准》轨道几何允许偏差；[北京 DB11/T 718-2016](https://jtw.beijing.gov.cn/gjsssb/zlybz/202111/P020211126384487884872.pdf) 表 8、16、22。本地副本在 `local_data/references/track/`。
- 镜头：[Schneider PYRITE 4.5/90 V38 数据表](https://schneiderkreuznach.com/download_file/force/3458/3860)，仅作畸变量级参考（§7.3）。

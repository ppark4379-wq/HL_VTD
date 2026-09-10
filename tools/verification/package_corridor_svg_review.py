"""Package a small, offline SVG gallery without PNGs or repeated full-map views."""
import argparse
import csv
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
NS = 'http://www.w3.org/2000/svg'
ET.register_namespace('', NS)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'output/콜리더_게이트웨이/자동생성_20개')
    parser.add_argument('--output', type=Path, default=ROOT/'output/콜리더_게이트웨이/SVG_검토')
    args = parser.parse_args()
    source, dest = args.source.resolve(), args.output.resolve()
    if dest.exists() and any(dest.iterdir()):
        parser.error('Use an empty output directory to preserve existing review files.')
    folders = sorted((p for p in source.iterdir() if p.is_dir() and p.name.startswith('A')
                      and p.name[1:].isdigit()), key=lambda p: int(p.name[1:]))
    if not folders:
        parser.error('No generated route folders found.')
    dest.mkdir(parents=True, exist_ok=True)
    selection = json.loads((source/'selection.json').read_text(encoding='utf-8'))
    kept = set(selection.get('kept', []))
    routes, all_points = [], []
    for folder in folders:
        # Namespace normalization removes repeated xmlns declarations only.
        # Preserve all coordinates, labels, paths, styles, and drawing layers.
        tree = ET.parse(folder/'overview.svg')
        svg = ET.tostring(tree.getroot(), encoding='unicode')
        assert ET.tostring(ET.fromstring(svg)) == ET.tostring(tree.getroot())
        (dest/f'{folder.name}.svg').write_text(svg, encoding='utf-8')
        csv_text = (folder/'checkpoints.csv').read_text(encoding='utf-8-sig')
        points = list(csv.DictReader(io.StringIO(csv_text)))
        all_points.extend(dict(scenario=folder.name, **point) for point in points)
        routes.append(dict(id=folder.name, kept=folder.name in kept, csv=csv_text,
                           gateways=(len(points)-2)//2))
    with (dest/'checkpoints.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['scenario', 'seq', 'x', 'y'])
        writer.writeheader(); writer.writerows(all_points)
    html = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>콜리더·게이트웨이 SVG 검토</title>
<style>body{margin:0;background:#152129;color:#fff;font:16px system-ui}header{position:sticky;top:0;background:#152129;padding:14px 24px;z-index:1;box-shadow:0 2px 8px #0008}select,button,a{font:inherit;margin-right:10px;padding:6px}a{color:#aee7ff}p{margin:8px 0}main{overflow:auto;padding:12px}img{display:block;width:100%;height:auto}h1{font-size:21px;margin:0 0 10px}</style>
<header><h1>콜리더·게이트웨이 __COUNT__개 · SVG 검토</h1>
<button id="previous">← 이전</button><select id="route" aria-label="경로"></select><button id="next">다음 →</button>
<button id="minus">축소 −</button><button id="plus">확대 +</button><button id="fit">화면 맞춤</button>
<a id="direct" target="_blank">SVG 원본</a><a id="csv">선택 경로 CSV</a><a href="checkpoints.csv" download>전체 좌표 CSV</a>
<p>노란 면: 콜리더 · 주황 윤곽: 게이트웨이 · P: 출발 / 교차로 진입·진출 / 도착</p></header>
<main><img id="map" alt="선택 경로의 콜리더 및 게이트웨이"></main>
<script>const routes=__ROUTES__;
const route=document.getElementById('route'),map=document.getElementById('map');let scale=100,csvURL=null;
for(const r of routes){const o=document.createElement('option');o.value=r.id;o.textContent=r.id+' · '+(r.kept?'보존':'새 경로')+' · 교차로 '+r.gateways+'쌍';route.appendChild(o);}
route.selectedIndex=Math.max(0,routes.findIndex(r=>!r.kept));
function refresh(){const r=routes[route.selectedIndex];map.src=r.id+'.svg';document.getElementById('direct').href=map.getAttribute('src');if(csvURL)URL.revokeObjectURL(csvURL);csvURL=URL.createObjectURL(new Blob([r.csv],{type:'text/csv;charset=utf-8'}));const a=document.getElementById('csv');a.href=csvURL;a.download=r.id+'_checkpoints.csv';scale=100;map.style.width='100%';}
route.onchange=refresh;
function step(d){route.selectedIndex=(route.selectedIndex+d+routes.length)%routes.length;refresh();}
document.getElementById('previous').onclick=()=>step(-1);document.getElementById('next').onclick=()=>step(1);
document.getElementById('plus').onclick=()=>{scale=Math.min(800,scale*1.4);map.style.width=scale+'%';};
document.getElementById('minus').onclick=()=>{scale=Math.max(25,scale/1.4);map.style.width=scale+'%';};
document.getElementById('fit').onclick=()=>{scale=100;map.style.width='100%';};refresh();</script></html>'''
    payload = json.dumps(routes, ensure_ascii=False).replace('<', '\\u003c')
    (dest/'index.html').write_text(html.replace('__COUNT__', str(len(routes))).replace('__ROUTES__', payload), encoding='utf-8')
    (dest/'README.md').write_text(f'''# 콜리더·게이트웨이 SVG 검토 {len(routes)}개

`index.html`을 브라우저로 열면 경로 선택·이전/다음·확대가 가능합니다. 인터넷 연결이나 서버 실행은 필요 없습니다. 개별 `Axx.svg`도 직접 열 수 있습니다.

보존 경로: {', '.join(sorted(kept))}. 나머지는 새로 생성한 경로입니다.

`checkpoints.csv`는 scenario,seq,x,y 형식의 전체 좌표입니다. 보기 화면의 '선택 경로 CSV'는 해당 경로의 원본 seq,x,y CSV를 내려받습니다.

기존 경로 SVG의 모든 좌표·차선 면·배경·라벨을 유지했습니다. XML 네임스페이스의 중복 선언만 정리했습니다. PNG, 경로마다 반복되는 전체 맵 SVG, 상세 JSON, 비교 스냅샷은 배포 묶음에서 제외했습니다. 로컬 원본 산출물은 별도로 유지합니다.

이 그림은 콜리더·게이트웨이 검토용이며 보간 및 차량 주행 검증 전 자료입니다. 합류·확장 차선 면 보완이 적용되어 있습니다.

전체 로컬 결과에서 이 묶음을 다시 만들려면 저장소 루트에서 다음을 실행합니다(새 출력 폴더 필요).
```powershell
python tools/verification/package_corridor_svg_review.py --output output/콜리더_게이트웨이/SVG_검토_새묶음
```
''', encoding='utf-8')
    files = [p for p in dest.iterdir() if p.is_file()]
    print(json.dumps(dict(routes=len(routes), files=len(files), bytes=sum(p.stat().st_size for p in files))))


if __name__ == '__main__':
    main()

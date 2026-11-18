import json
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4,landscape
R=Path('/home/pa4/git/lv2_next_module');o=R/'Cartlidar/sim_render/results/camera_calibration_261029';d=json.loads((o/'board_vector_data.json').read_text());p=o/'output/pdf/charuco_monitor_10x7_A4.pdf';c=canvas.Canvas(str(p),pagesize=landscape(A4));W,H=landscape(A4);bw,bh=250*mm,175*mm;x0,y0=(W-bw)/2,(H-bh)/2
c.setTitle('ChArUco calibration board - 10x7 - DICT_5X5_100')
c.setAuthor('');c.setFillColorRGB(1,1,1);c.rect(0,0,W,H,fill=1,stroke=0);c.setFillColorRGB(0,0,0)
for x,y in d['black_cells']:c.rect(x0+x*25*mm,y0+bh-(y+1)*25*mm,25*mm,25*mm,fill=1,stroke=0)
for m in d['markers']:
 points=m['corners_mm'];left,top=points[0][:2];size=points[1][0]-left
 for y,row in enumerate(m['bits']):
  for x,bit in enumerate(row):
   if bit:c.rect(x0+(left+x*size/7)*mm,y0+bh-(top+(y+1)*size/7)*mm,size/7*mm,size/7*mm,fill=1,stroke=0)
c.setFont('Helvetica',9);c.drawCentredString(W/2,H-8*mm,'ChArUco 10 x 7 squares | DICT_5X5_100 | square 25 mm, marker 18 mm at 100%')
c.setFont('Helvetica',8);c.drawCentredString(W/2,7*mm,'MONITOR: measure the displayed square size and keep zoom fixed. PRINT: A4 landscape, actual size / 100%.')
c.showPage();c.save();print(p)

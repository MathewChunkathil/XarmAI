"""Printable tag36h11 sheet. Print at 100% (no fit-to-page). Black square edge = 40 mm."""
import cv2, numpy as np, os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
TAG_MM, IDS = 40, [0, 1, 2, 3, 4]   # cube faces: 5 ids
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
os.makedirs("results/tags", exist_ok=True)
c = canvas.Canvas("results/tags/tag36h11_40mm.pdf", pagesize=A4)
W, H = A4
c.setFont("Helvetica", 9)
c.drawString(15*mm, H-12*mm, f"tag36h11, black square edge = {TAG_MM} mm. Print at 100% scale. Verify with ruler.")
for n, i in enumerate(IDS):
    png = f"results/tags/tag36h11_id{i}.png"
    cv2.imwrite(png, cv2.aruco.generateImageMarker(d, i, 800))
    col, row = n % 3, n // 3
    x, y = 25*mm + col*60*mm, H - 70*mm - row*70*mm
    c.drawImage(png, x, y, TAG_MM*mm, TAG_MM*mm)
    c.drawString(x, y-5*mm, f"id {i}  ({TAG_MM} mm)")
    c.rect(x-5*mm, y-8*mm, (TAG_MM+10)*mm, (TAG_MM+16)*mm)   # cut guide / white margin
c.save(); print("ok")

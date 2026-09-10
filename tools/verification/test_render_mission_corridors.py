import unittest
import xml.etree.ElementTree as ET
import numpy as np
from PyQt5.QtCore import QByteArray, QRectF
from PyQt5.QtGui import QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer
from render_mission_corridors import RoadBackground


class SurfaceRenderingTests(unittest.TestCase):
    def render(self, content):
        svg = f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">{content}</svg>'
        renderer = QSvgRenderer(QByteArray(svg.encode()))
        image = QImage(100, 100, QImage.Format_ARGB32)
        image.fill(0)
        painter = QPainter(image)
        renderer.render(painter, QRectF(0, 0, 100, 100))
        painter.end()
        return image

    def background(self):
        bg = RoadBackground()
        bg.driving[(1, 0, -1)] = bg.path([(10,10),(90,10),(90,90),(10,90)])
        # Another road's sidewalk overlaps this connector. It must remain clear.
        bg.sidewalks.append(bg.path([(10,30),(30,30),(30,70),(10,70)]))
        return bg

    def test_fill_and_gateway_outline_do_not_color_sidewalk_or_outside_road(self):
        bg = self.background()
        vertices = np.array([(5,-10),(90,-10),(90,-90),(5,-90)])
        image = self.render(bg.polygon(vertices, (1,0,-1), fill='#fff000', stroke='#ff8500', stroke_width=12))
        for point in [(5,50),(20,50),(95,50),(50,5),(50,95)]:
            self.assertEqual(image.pixelColor(*point).alpha(), 0, point)
        self.assertGreater(image.pixelColor(50,50).alpha(), 0)
        self.assertEqual(image.pixelColor(87,50).name(), '#ff8500')

    def test_gateway_interface_stroke_stays_in_driving_surface(self):
        image = self.render(self.background().line([(0,-50),(100,-50)], (1,0,-1), 'white', 8))
        for point in [(5,50),(20,50),(95,50)]:
            self.assertEqual(image.pixelColor(*point).alpha(), 0, point)
        self.assertEqual(image.pixelColor(50,50).name(), '#ffffff')

    def test_sidewalk_hole_is_preserved(self):
        bg = self.background()
        bg.sidewalks.append(bg.path([(40,40),(60,40),(60,60),(40,60)]))
        image = self.render(bg.polygon(np.array([(10,-10),(90,-10),(90,-90),(10,-90)]), (1,0,-1), fill='yellow'))
        self.assertEqual(image.pixelColor(50,50).alpha(), 0)
        self.assertGreater(image.pixelColor(70,50).alpha(), 0)


if __name__ == '__main__':
    unittest.main()

"""
Macro de FreeCAD: modela la silla objetivo y exporta sus mallas para Gazebo (research R4).

Ejecutar con la interfaz (Macro > Ejecutar), por el servidor MCP de FreeCAD o sin interfaz:

    freecadcmd percepcion_nav/freecad/make_chair.py

Versión 2. La versión 1 era un bloque de cajas de un solo color y YOLO11n no la reconocía como
silla (score máximo de 0.09; docs/validacion_simulacion.md). Esta versión imita los rasgos que
sí reconoce: una butaca con estructura de madera (patas, brazos y travesaños) y cojines de
tela con bordes redondeados, cada parte con su propio color.

Dimensiones en milímetros. El origen está en el centro de la huella, a nivel del piso, y el
frente mira hacia +X. Huella de 520 (fondo) x 560 (ancho), altura de 900.

- Patas de 40 x 40 x 620 en las esquinas; brazos de 540 x 50 x 30 sobre ellas.
- Travesaños laterales a 300–340 y travesaño frontal a 280–330, al ras del frente: junto con el
  frente del cojín del asiento (330–440) cubren la ventana central de profundidad, de modo que
  la mediana cae en la silla y no en el fondo entre las patas (R10).
- Cojín del asiento de 480 x 480 x 110; cojín del respaldo de 100 x 480 x 460, inclinado 10°
  hacia atrás.

Salidas: models/freecad_chair/meshes/chair_frame.stl y chair_cushions.stl (en mm; el SDF las
escala a metros) y freecad/chair.FCStd (documento editable).
"""

import os

import FreeCAD as App  # noqa: I100 - módulos de FreeCAD
import Mesh
import MeshPart
import Part

DEPTH = 520.0
WIDTH = 560.0
LEG = 40.0
LEG_HEIGHT = 620.0
ARM_WIDTH = 50.0
ARM_THICKNESS = 30.0
ARM_OVERHANG = 20.0
SIDE_RAIL = (300.0, 340.0)
FRONT_RAIL = (280.0, 330.0)
SEAT = {'back': -230.0, 'front': 250.0, 'half_width': 240.0, 'bottom': 330.0, 'top': 440.0}
BACKREST = {'back': -250.0, 'thickness': 100.0, 'height': 460.0, 'tilt_deg': -10.0}
CUSHION_FILLET = 25.0

HALF_D = DEPTH / 2.0
HALF_W = WIDTH / 2.0


def package_dir():
    """Directorio del paquete percepcion_nav (este archivo vive en percepcion_nav/freecad/)."""
    env_dir = os.environ.get('PERCEPCION_NAV_DIR')
    if env_dir:
        return os.path.abspath(env_dir)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def box(x0, x1, y0, y1, z0, z1):
    """Caja alineada a los ejes entre las coordenadas dadas, en mm."""
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, App.Vector(x0, y0, z0))


def build_frame():
    """Estructura de madera: patas, brazos y travesaños."""
    xs = ((HALF_D - LEG, HALF_D), (-HALF_D, -HALF_D + LEG))
    ys = ((HALF_W - LEG, HALF_W), (-HALF_W, -HALF_W + LEG))
    parts = [box(x0, x1, y0, y1, 0.0, LEG_HEIGHT) for x0, x1 in xs for y0, y1 in ys]
    for y0, y1 in ((HALF_W - ARM_WIDTH, HALF_W), (-HALF_W, -HALF_W + ARM_WIDTH)):
        parts.append(box(-HALF_D, HALF_D + ARM_OVERHANG, y0, y1,
                         LEG_HEIGHT, LEG_HEIGHT + ARM_THICKNESS))
    for y0, y1 in ys:
        parts.append(box(-HALF_D + LEG, HALF_D - LEG, y0, y1, *SIDE_RAIL))
    parts.append(box(HALF_D - LEG, HALF_D, -HALF_W + LEG, HALF_W - LEG, *FRONT_RAIL))
    return parts[0].fuse(parts[1:]).removeSplitter()


def build_cushions():
    """Cojines de tela con bordes redondeados: asiento y respaldo inclinado."""
    seat = box(SEAT['back'], SEAT['front'], -SEAT['half_width'], SEAT['half_width'],
               SEAT['bottom'], SEAT['top'])
    seat = seat.makeFillet(CUSHION_FILLET, seat.Edges)
    back_x0 = BACKREST['back']
    backrest = box(back_x0, back_x0 + BACKREST['thickness'], -SEAT['half_width'],
                   SEAT['half_width'], SEAT['top'], SEAT['top'] + BACKREST['height'])
    backrest = backrest.makeFillet(CUSHION_FILLET, backrest.Edges)
    backrest.rotate(App.Vector(back_x0, 0.0, SEAT['top']), App.Vector(0, 1, 0),
                    BACKREST['tilt_deg'])
    return seat.fuse(backrest)


def export_mesh(shape, path):
    """Teselado fino para que los bordes redondeados se vean suaves en el render."""
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.5, AngularDeflection=0.2,
                                  Relative=False)
    Mesh.Mesh(mesh).write(path)
    return mesh.CountFacets


def main():
    root = package_dir()
    mesh_dir = os.path.join(root, 'models', 'freecad_chair', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)

    doc = App.newDocument('silla_objetivo')
    frame = doc.addObject('Part::Feature', 'Estructura')
    frame.Shape = build_frame()
    cushions = doc.addObject('Part::Feature', 'Cojines')
    cushions.Shape = build_cushions()
    doc.recompute()
    doc.saveAs(os.path.join(root, 'freecad', 'chair.FCStd'))

    n_frame = export_mesh(frame.Shape, os.path.join(mesh_dir, 'chair_frame.stl'))
    n_cushions = export_mesh(cushions.Shape, os.path.join(mesh_dir, 'chair_cushions.stl'))
    bound = frame.Shape.BoundBox
    bound.add(cushions.Shape.BoundBox)
    print(f'Silla exportada: estructura {n_frame} y cojines {n_cushions} triángulos; caja '
          f'{bound.XLength:.0f} x {bound.YLength:.0f} x {bound.ZLength:.0f} mm.')


main()

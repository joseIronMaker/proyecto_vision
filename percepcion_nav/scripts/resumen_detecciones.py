"""Resume un mensaje vision_msgs/Detection2DArray leído de `ros2 topic echo --once` (stdin)."""

import sys

import yaml


def main():
    text = sys.stdin.read().split('---')[0]
    if not text.strip():
        print('  (no llegó ningún mensaje de /detections)')
        return
    msg = yaml.safe_load(text)
    stamp = msg['header']['stamp']
    print(f"  header: stamp {stamp['sec']}.{stamp['nanosec']:09d}  "
          f"frame_id {msg['header']['frame_id']}")
    for det in msg['detections'][:5]:
        bbox = det['bbox']
        hyp = det['results'][0]['hypothesis']
        center = bbox['center']['position']
        print(f"  - {hyp['class_id']:<10} score {hyp['score']:.2f}  "
              f"bbox centro ({center['x']:.0f}, {center['y']:.0f}) px  "
              f"tamaño {bbox['size_x']:.0f} x {bbox['size_y']:.0f} px")


if __name__ == '__main__':
    main()

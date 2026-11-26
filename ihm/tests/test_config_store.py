from pathlib import Path
import tempfile
import unittest

from ihm.config_store import load_config, load_field_size, save_config


SAMPLE = """%YAML:1.0
---
color:
    hue_tol: 5
    sat_tol: 75
    rectMinArea: 200.0
    circleMinArea: 5.0
    ballMinArea: 15.0
    roiWidth: 60
    roiHeight: 60

camera:
    id: 0
    width: 1280
    height: 720
    fps: 30

window:
    name: window
    delay: 34

field:
    sizeX_cm: 237.0
    sizeY_cm: 165.0
"""


class ConfigStoreTests(unittest.TestCase):
    def test_round_trip_preserves_opencv_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(SAMPLE, encoding="utf-8")
            config = load_config(path)
            self.assertEqual(config.width, 1280)
            config.fps = 60
            config.hue_tol = 9
            save_config(path, config)
            result = load_config(path)
            self.assertEqual(result.fps, 60)
            self.assertEqual(result.hue_tol, 9)
            self.assertTrue(
                path.read_text(encoding="utf-8").startswith("%YAML:1.0")
            )

    def test_field_size_survives_saving_camera_size(self):
        # field e camera nao podem se confundir: as chaves tem nomes diferentes
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(SAMPLE, encoding="utf-8")
            self.assertEqual(load_field_size(path), (237.0, 165.0))
            config = load_config(path)
            config.width = 640
            config.height = 480
            save_config(path, config)
            self.assertEqual(load_field_size(path), (237.0, 165.0))
            self.assertEqual(load_config(path).width, 640)

    def test_field_size_defaults_when_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(SAMPLE.split("field:")[0], encoding="utf-8")
            self.assertEqual(load_field_size(path), (237.0, 165.0))


if __name__ == "__main__":
    unittest.main()

import unittest
from types import SimpleNamespace

from identify_pack.scripts.trim_gate import should_publish_trim


class TrimGateTests(unittest.TestCase):
    def test_requires_detection(self):
        self.assertFalse(should_publish_trim(SimpleNamespace(
            detected=False, stable=True, confidence=1.0)))

    def test_requires_stable_by_default(self):
        self.assertFalse(should_publish_trim(SimpleNamespace(
            detected=True, stable=False, confidence=0.9)))
        self.assertTrue(should_publish_trim(SimpleNamespace(
            detected=True, stable=True, confidence=0.9)))

    def test_can_disable_stable_requirement(self):
        self.assertTrue(should_publish_trim(SimpleNamespace(
            detected=True, stable=False, confidence=0.9),
            require_stable=False))

    def test_confidence_threshold(self):
        result = SimpleNamespace(detected=True, stable=True, confidence=0.4)
        self.assertFalse(should_publish_trim(result, minimum_confidence=0.5))
        self.assertTrue(should_publish_trim(result, minimum_confidence=0.4))
        # 没有 confidence 字段时不做该门槛，只依赖 stable
        self.assertTrue(should_publish_trim(SimpleNamespace(
            detected=True, stable=True)))


if __name__ == '__main__':
    unittest.main()

import unittest

from macgyvbot_config.drawer import (
    DRAWER_COLLISION_BOX_PROFILES,
    DRAWER_COLLISION_DEFAULT_PROFILE,
    DRAWER_COLLISION_SCENE_KEY_PROFILES,
)


class DrawerCollisionSceneConfigTest(unittest.TestCase):
    def test_default_profile_uses_static_drawer_body_only(self):
        boxes = DRAWER_COLLISION_BOX_PROFILES[DRAWER_COLLISION_DEFAULT_PROFILE]
        object_ids = {box["id"] for box in boxes}

        self.assertEqual(object_ids, {"drawer_body_boundary"})
        self.assertNotIn("drawer_opened_boundary", object_ids)

    def test_motion_key_routing_remains_available_but_unregistered(self):
        self.assertEqual(DRAWER_COLLISION_SCENE_KEY_PROFILES, {})


if __name__ == "__main__":
    unittest.main()

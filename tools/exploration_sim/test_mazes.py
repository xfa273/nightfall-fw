"""Ground-truth parsing checks; no download or external clone is required."""
import tempfile
from pathlib import Path
import unittest

try:
    from .mazes import MazeFormatError, catalog, load_maze, parse_maze
except ImportError:
    from mazes import MazeFormatError, catalog, load_maze, parse_maze


# Asymmetric geometry catches swapped directions and a north/south inversion.
SMALL = """+---+---+
| G | G |
+   +   +
| S     |
+---+---+
"""


class MazeParserTests(unittest.TestCase):
    def test_coordinate_orientation_shared_walls_and_all_goal_markers(self):
        maze = parse_maze(SMALL)
        self.assertEqual((maze.width, maze.height), (2, 2))
        self.assertEqual(maze.start, (0, 0))
        self.assertEqual(maze.goals, [(0, 1), (1, 1)])
        self.assertEqual(maze.walls, [[12, 6], [11, 11]])
        self.assertEqual(maze.metadata['reachable_cells'], 4)
        self.assertEqual(maze.to_dict()['goals'], [[0, 1], [1, 1]])

    def test_unknown_source_wall_is_not_treated_as_an_open_passage(self):
        with self.assertRaisesRegex(MazeFormatError, "unknown"):
            parse_maze(SMALL.replace("+   +   +", "+ . +   +"))

    def test_crlf_and_missing_terminal_newline(self):
        self.assertEqual(parse_maze(SMALL.rstrip().replace('\n', '\r\n')).walls,
                         parse_maze(SMALL).walls)

    def test_boundaries_and_goals_are_validated(self):
        for text, reason in [
            (SMALL.replace('+---+---+', '+   +---+', 1), 'outside boundary'),
            (SMALL.replace(' G ', '   '), 'No G markers'),
            (SMALL.replace('+   +   +', '+---+---+'), 'reachable'),
            (SMALL.replace(' S ', ' G '), 'exactly one S'),
            (SMALL.replace('| S     |', '| S    |'), 'inconsistent widths'),
        ]:
            with self.subTest(reason=reason), self.assertRaisesRegex(MazeFormatError, reason):
                parse_maze(text)

    def test_catalog_keeps_invalid_source_visible_and_records_content_hash(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'data').mkdir()
            (root / 'data/16MM2023HX.maze').write_text(SMALL)
            (root / 'data/16MM2022HX.maze').write_text(SMALL.replace('+   +   +', '+ . +   +'))
            entries = catalog(root, download=False)
            self.assertEqual(len(entries), 2)
            self.assertTrue(entries[0]['available'])
            self.assertFalse(entries[1]['available'])
            self.assertIn('unknown', entries[1]['error'])
            self.assertEqual(entries[0]['source_revision'], 'local-unversioned')
            self.assertEqual(len(entries[0]['source_sha256']), 64)
            self.assertEqual(load_maze('16MM2023HX', root).goals, [(0, 1), (1, 1)])
            with self.assertRaises(ValueError):
                load_maze('../16MM2023HX', root)
            with self.assertRaises(FileNotFoundError):
                catalog(root / 'missing', download=False)


if __name__ == '__main__':
    unittest.main()

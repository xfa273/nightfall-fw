import tempfile
from pathlib import Path
import subprocess
import unittest

from tools.exploration_sim.engine import Knowledge
from tools.exploration_sim.mcu_benchmark_cases import export_cases, firmware_map, header_text


def tiny_replay():
    truth = [[12, 6], [9, 3]]
    knowledge = Knowledge(2, 2)
    events = []
    for step, (x, y, heading) in enumerate(((0, 0, 0), (0, 1, 0), (1, 1, 1))):
        changes = knowledge.observe(truth, (x, y), heading)
        events.append(dict(step=step, x=x, y=y, heading=heading, t=float(step), dt=1.0,
                           changes=changes, known_edges=knowledge.count,
                           certified=step == 2, lower_s=0.1, upper_s=0.1 if step == 2 else None))
    return {"maze": {"id": "tiny", "width": 2, "height": 2, "walls": truth, "goals": [[0, 1]]},
            "profile": {"machine": "mini_r2_0", "cell_mm": 90, "search_speed_mm_s": 400},
            "runs": [{"algorithm": "relevant", "events": events}], "options": {"max_steps": 12000}}


class MCUFixtureTests(unittest.TestCase):
    def test_partial_map_never_uses_final_truth(self):
        result = tiny_replay()
        corpus = export_cases(result)
        cases = {case["stage"]: case for case in corpus["cases"]}
        self.assertEqual(set(cases), {"initial", "first_goal", "uncertain_mid", "certificate", "full_truth"})
        self.assertFalse(cases["initial"]["known"][0][1] & 1)
        self.assertTrue(cases["full_truth"]["known"][0][1] & 1)
        self.assertTrue(cases["full_truth"]["evaluation_only"])
        self.assertFalse(cases["certificate"]["evaluation_only"])
        self.assertFalse(cases["full_truth"]["visited"][0][1])
        self.assertEqual(cases["initial"]["optimistic"][0][1] & 1, 0)
        self.assertEqual(cases["initial"]["conservative"][0][1] & 1, 1)
        for case in corpus["cases"]:
            self.assertEqual(case["firmware_map_u8"], firmware_map(case["known"], case["walls"]))

    def test_known_open_unknown_and_wall_have_distinct_firmware_encoding(self):
        self.assertEqual(firmware_map([[0]], [[0]]), [[0xF0]])
        self.assertEqual(firmware_map([[15]], [[0]]), [[0x00]])
        self.assertEqual(firmware_map([[15]], [[1]]), [[0x88]])
        self.assertEqual(firmware_map([[15]], [[2]]), [[0x44]])
        self.assertEqual(firmware_map([[15]], [[4]]), [[0x22]])
        self.assertEqual(firmware_map([[15]], [[8]]), [[0x11]])

    def test_missing_certification_is_reported(self):
        result = tiny_replay()
        for event in result["runs"][0]["events"]:
            event["certified"] = False
        corpus = export_cases(result, include_truth=False)
        self.assertIn("certificate", corpus["missing_stages"])
        self.assertFalse(any(case["evaluation_only"] for case in corpus["cases"]))

    def test_inconsistent_replay_is_rejected(self):
        result = tiny_replay()
        result["runs"][0]["events"][0]["known_edges"] += 1
        with self.assertRaisesRegex(ValueError, "counter"):
            export_cases(result)
        result = tiny_replay()
        result["runs"][0]["events"][0]["changes"][0][3] ^= 1
        with self.assertRaisesRegex(ValueError, "truth"):
            export_cases(result)

    def test_generated_header_has_self_contained_typed_fixtures(self):
        header = header_text(export_cases(tiny_replay()))
        self.assertIn("const uint8_t *known, *walls, *visited, *goals, *firmware_map", header)
        self.assertIn("NF_MCU_BENCHMARK_CASE_COUNT", header)
        self.assertIn('"tiny_relevant_full_truth"', header)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'cases.h').write_text(header)
            source = directory / 'check.c'
            source.write_text('#include "cases.h"\nint main(void) { return NF_MCU_BENCHMARK_CASE_COUNT == 5 && nf_mcu_benchmark_cases[4].evaluation_only ? 0 : 1; }\n')
            binary = directory / 'check'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary)], check=True)

            source.write_text('#define NF_BENCH_CASE_INDEX 4\n#include "cases.h"\nint main(void) { return NF_MCU_BENCHMARK_CASE_COUNT == 1 && nf_mcu_benchmark_cases[0].evaluation_only ? 0 : 1; }\n')
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary)], check=True)
            source.write_text('#define NF_BENCH_CASE_INDEX 5\n#include "cases.h"\n')
            invalid = subprocess.run(['cc', '-std=c11', '-c', str(source), '-o', str(directory / 'invalid.o')], capture_output=True, text=True)
            self.assertNotEqual(invalid.returncode, 0)
            self.assertIn('outside the generated fixture corpus', invalid.stderr)


if __name__ == '__main__':
    unittest.main()

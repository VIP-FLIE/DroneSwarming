"""Check diagnostic integrity without ROS, PX4, or application processes."""

from __future__ import annotations

import json
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree
from pathlib import Path

from run import JsonValue, recent_events, reports, write_json


class DiagnosticReports(unittest.TestCase):
    """Keep crashes, timeouts, and failed contracts distinguishable in retained files."""

    def test_junit_preserves_failure_classification(self) -> None:
        """A timeout or process error must never become a passing JUnit case."""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results: list[dict[str, JsonValue]] = [
                {
                    'case': status,
                    'seed': 7,
                    'status': status,
                    'elapsed_sec': 0.5,
                    'artifact_directory': status,
                    'errors': ['error detail'],
                    'failures': ['contract <threshold>'],
                }
                for status in ('passed', 'failed', 'error', 'timeout')
            ]
            reports(directory, results)
            junit: ElementTree.Element = ElementTree.parse(directory / 'junit.xml').getroot()
            self.assertEqual(junit.attrib['failures'], '1')
            self.assertEqual(junit.attrib['errors'], '2')
            self.assertEqual(len(junit.findall('testcase/failure')), 1)
            self.assertEqual(len(junit.findall('testcase/error')), 2)
            self.assertEqual(
                json.loads((directory / 'summary.json').read_text())['counts']['passed'], 1
            )

    def test_partial_timeline_keeps_complete_events(self) -> None:
        """Recover recent complete JSON records around a truncated crash record."""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'events.jsonl').write_text(
                '{"sequence": 1}\n{"sequence": 2}\n{"seq', encoding='utf-8'
            )
            self.assertEqual(recent_events(directory), [{'sequence': 1}, {'sequence': 2}])

    def test_invalid_numeric_report_preserves_previous_record(self) -> None:
        """Refuse non-standard JSON without corrupting the last complete result."""
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'result.json'
            write_json(path, {'status': 'failed'})
            with self.assertRaises(ValueError):
                write_json(path, {'metric': float('nan')})
            self.assertEqual(json.loads(path.read_text()), {'status': 'failed'})


if __name__ == '__main__':
    unittest.main()

"""Startup provenance must fail closed without affecting the runtime."""

import os
import subprocess
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.config import load_config
from core.runtime_diagnostics import source_revision
from runtime import CaptainRuntime


class SourceRevisionTests(unittest.TestCase):
    project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    commit = 'a' * 40

    def responses(self, branch='fix/test', status=''):
        return [self.project.encode('utf-8'), self.commit.encode('ascii'),
                branch.encode('utf-8'), status.encode('utf-8')]

    def test_clean_dirty_and_detached_checkouts_are_reported_without_mutations(self):
        for branch, status in (('main', ''), ('fix/test', ' M runtime.py\n?? test.py'),
                               ('HEAD', '')):
            with self.subTest(branch=branch, status=status):
                with patch('core.runtime_diagnostics.subprocess.check_output',
                           side_effect=self.responses(branch, status)) as run:
                    result = source_revision(self.project)
                self.assertEqual('captured', result['status'])
                self.assertEqual(self.commit, result['git_commit'])
                self.assertEqual(branch if branch != 'HEAD' else None, result['git_branch'])
                self.assertEqual(bool(status), result['dirty'])
                self.assertEqual(['rev-parse', 'rev-parse', 'rev-parse', 'status'],
                                 [call[0][0][1] for call in run.call_args_list])
                for call in run.call_args_list:
                    self.assertFalse(call[1].get('shell', False))
                    self.assertEqual(self.project, call[1]['cwd'])
                    self.assertEqual(1.0, call[1]['timeout'])

    def test_parent_repository_cannot_identify_an_exported_project(self):
        with patch('core.runtime_diagnostics.subprocess.check_output',
                   return_value=os.path.dirname(self.project).encode('utf-8')) as run:
            result = source_revision(self.project)
        self.assertEqual('unavailable', result['status'])
        self.assertIsNone(result['git_commit'])
        self.assertEqual(1, run.call_count)

    def test_missing_git_timeout_and_failed_status_do_not_report_a_verified_commit(self):
        errors = [OSError('missing git'), subprocess.TimeoutExpired(['git'], 1),
                  subprocess.CalledProcessError(1, ['git'])]
        for error in errors:
            for after in (0, 3):
                with self.subTest(error=type(error).__name__, after=after):
                    responses = self.responses()[:after] + [error]
                    with patch('core.runtime_diagnostics.subprocess.check_output',
                               side_effect=responses):
                        result = source_revision(self.project)
                    self.assertEqual('unavailable', result['status'])
                    self.assertIsNone(result['git_commit'])
                    self.assertIsNone(result['dirty'])

    def test_probe_runs_once_before_sdk_bootstrap_and_never_during_construction(self):
        metadata = {'status': 'captured', 'git_commit': self.commit,
                    'git_branch': 'fix/test', 'dirty': False}
        with tempfile.TemporaryDirectory() as directory:
            config = load_config(self.project)
            config.runtime_dir = directory
            logger = SimpleNamespace(info=lambda *args: None, warning=lambda *args: None)
            with patch('runtime.source_revision', return_value=metadata) as probe:
                runtime = CaptainRuntime(config, logger)
                probe.assert_not_called()
                def bootstrap():
                    self.assertEqual(metadata, runtime.runtime_info['source'])
                    raise ValueError('SDK fixture')
                with patch.object(runtime, '_install_signals'), \
                     patch.object(runtime.adapter, 'bootstrap', side_effect=bootstrap), \
                     patch.object(runtime.adapter, 'shutdown'):
                    with self.assertRaisesRegex(ValueError, 'SDK fixture'):
                        runtime.run()
                probe.assert_called_once_with(self.project)
            self.assertFalse(os.path.exists(runtime.pid_path))
            self.assertIsNone(runtime._lock_stream)

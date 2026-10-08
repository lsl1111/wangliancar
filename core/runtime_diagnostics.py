"""Read-only startup provenance and brake evidence, independent of the SDK."""

import os
import re
import subprocess


def source_revision(project_dir):
    """Identify the startup checkout; exports without Git remain unknown.

    Ignore Git's stderr (it may contain remote details) and bound each probe.
    This runs once at startup, never in the control loop. A dirty tree does
    not assert that the running source equals the recorded commit.
    """
    result = {"status": "unavailable", "git_commit": None,
              "git_branch": None, "dirty": None}

    def git(*args):
        return subprocess.check_output(
            ["git"] + list(args), cwd=project_dir,
            stderr=subprocess.DEVNULL, timeout=1.0,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)
        ).decode("utf-8", errors="replace").strip()

    try:
        root = git("rev-parse", "--show-toplevel")
        if os.path.normcase(os.path.realpath(root)) != os.path.normcase(
                os.path.realpath(project_dir)):
            # A deployed export inside another repository is not that repo's
            # version. Do not accidentally attribute its parent's revision.
            return result
        commit = git("rev-parse", "HEAD")
        if re.fullmatch(r"[0-9a-fA-F]{40}", commit) is None:
            return result
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        dirty = bool(git("status", "--porcelain", "--untracked-files=normal"))
    except (OSError, subprocess.SubprocessError):
        return result
    result.update(status="captured", git_commit=commit,
                  git_branch=None if branch == "HEAD" else branch, dirty=dirty)
    return result


class BrakeDiagnostics(object):
    """Separate requests, successful sends and observed GPS feedback.

    None means there is no new usable candidate. It must not be interpreted
    as a zero-brake command. A successful send is still not execution proof.
    """

    def __init__(self):
        self.last_successful_command = None

    def observe(self, perception, trajectory, control, receipt, safety, now):
        candidate = safety.control if safety.active else control
        available = candidate is not None and candidate.valid is True
        braking = bool(candidate.brake > 0 or candidate.handbrake) if available else None
        requested = bool(safety.active or not trajectory.valid or trajectory.emergency_stop
                         or braking is True)
        previous = self.last_successful_command
        sent = available and receipt.get("attempted") is True and receipt.get("ok") is True
        release_sent = bool(sent and braking is False and previous is not None
                            and previous["braking"] is True)
        if sent:
            self.last_successful_command = {
                "frame_id": candidate.frame_id, "recorded_monotonic": now,
                "source": candidate.source, "braking": braking,
                "brake": candidate.brake, "handbrake": candidate.handbrake,
                "throttle": candidate.throttle}
        return {"requested": requested, "candidate_available": available,
                "candidate_braking": braking, "send_succeeded": sent,
                "release_sent": release_sent,
                "last_successful_command": (dict(self.last_successful_command)
                                            if self.last_successful_command is not None else None),
                "observed_brake": perception.ego.brake,
                "observed_frame_id": perception.ego.frame_id,
                "observed_valid": perception.valid is True and perception.ego.valid is True}

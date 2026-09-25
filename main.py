from __future__ import print_function

import argparse
import os
import sys

from core.config import load_config
from core.logging_setup import setup_logging
from runtime import CaptainRuntime
from simone_platform.simone_adapter import SimOneAdapter


PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    parser = argparse.ArgumentParser(description="NEVC 队长/API/感知集成程序")
    parser.add_argument("--config", help="叠加的本机 INI 配置")
    parser.add_argument("--once", action="store_true", help="案例运行后只处理一帧")
    parser.add_argument(
        "--self-test", action="store_true", help="只验证配置和 SDK 是否可加载，不连接案例"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_config(PROJECT_DIR, args.config)
    logger = setup_logging(config.runtime_dir, config.log_level)
    if args.self_test:
        adapter = SimOneAdapter(config, logger)
        adapter.bootstrap()
        print("SELF_TEST_OK: 配置、Python 依赖与 SimOne SDK 均可加载")
        return 0
    runtime = CaptainRuntime(config, logger)
    try:
        runtime.run(once=args.once)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        logger.exception("队长程序异常退出: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())


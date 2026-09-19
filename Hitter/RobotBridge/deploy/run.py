import os
import sys
import fcntl
from contextlib import contextmanager
from pathlib import Path

import hydra
from hydra.core.hydra_config import HydraConfig
from hydra.utils import instantiate, get_class
from omegaconf import DictConfig

from loguru import logger

DEPLOY_DIR = Path(__file__).resolve().parent
BRIDGE_DIR = DEPLOY_DIR.parent
CONFIG_DIR = str((DEPLOY_DIR / "config").resolve())
POLICY_LOCK_PATH = DEPLOY_DIR / "logs" / "policy_runtime.lock"

for path in (DEPLOY_DIR, BRIDGE_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


@contextmanager
def acquire_single_instance_lock(lock_path: Path = POLICY_LOCK_PATH):
    lock_path = Path(lock_path)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = lock_path.open("a+")
    try:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock_file.seek(0)
            holder = lock_file.read().strip() or "unknown pid"
            raise RuntimeError(
                f"Another RobotBridge policy process is already running ({holder}). "
                "Stop it before launching a new policy."
            ) from exc

        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(f"pid={os.getpid()}\n")
        lock_file.flush()
        yield lock_file
    finally:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            lock_file.close()

@hydra.main(
    version_base=None,
    config_path=CONFIG_DIR,
    config_name="hitter",
)
def main(cfg: DictConfig) -> None:
    os.chdir(DEPLOY_DIR)
    hydra_log_path = os.path.join(HydraConfig.get().runtime.output_dir, 'eval.log')
    logger.remove()
    logger.add(hydra_log_path, level='DEBUG')

    console_log_level = os.environ.get('LOGURU_LEVEL', 'INFO').upper()
    logger.add(sys.stdout, level=console_log_level, colorize=True)
    logger.info(f'Log saved to {hydra_log_path}')

    with acquire_single_instance_lock():
        agent = instantiate(cfg.agent)
        agent.run()


if __name__=="__main__":
    main()

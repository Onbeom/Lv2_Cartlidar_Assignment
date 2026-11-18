# 진단용(이 폴더에서만): SIGUSR1 을 받으면 모든 스레드 스택을 stderr 로 씀
import faulthandler, signal
faulthandler.register(signal.SIGUSR1, all_threads=True)

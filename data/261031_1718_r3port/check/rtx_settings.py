"""렌더 기본 노출 · 톤매핑 설정 읽기(바꾸지 않음)."""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "anti_aliasing": 0})
import carb
s = carb.settings.get_settings()
for k in ("/rtx/post/histogram/enabled", "/rtx/post/histogram/whiteScale", "/rtx/post/histogram/minEV", "/rtx/post/histogram/maxEV",
          "/rtx/post/tonemap/op", "/rtx/post/tonemap/filmIso", "/rtx/post/tonemap/fNumber", "/rtx/post/tonemap/cameraShutter",
          "/rtx/post/tonemap/enableSrgbToGamma", "/rtx/post/autoExposure/enabled", "/rtx/rendermode"):
    print("SET", k, s.get(k))
app.close()

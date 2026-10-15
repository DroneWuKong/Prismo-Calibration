import json
from pathlib import Path
import re
import subprocess


def test_browser_preset_math_for_panel_sizes_and_windows_scaling():
    html=(Path(__file__).resolve().parents[1]/'calibration_app.html').read_text()
    script=re.search(r'<script>(.*?)</script>',html,re.S).group(1)
    prefix=script.split('let ppm=',1)[0]
    code=prefix+"\nconst out={};for(const k of Object.keys(DISPLAY_PRESETS)){out[k]=[1,1.5,2,3].map(r=>presetScale(k,r));}console.log(JSON.stringify(out));"
    results=json.loads(subprocess.check_output(['node','-e',code],text=True))
    # Nominal active widths calculated from the published diagonals/aspect ratios.
    panels={'px13':(2880,286.5),'z13-qhd':(2560,288.6),'z13-fhd':(1920,288.6),
            'z13-uhd':(3840,288.6),'g14-oled':(2880,301.6),'g14-qhd':(2560,301.6),
            'g14-fhd':(1920,301.6),'g14-legacy':(2560,309.9)}
    assert set(results)==set(panels)
    for name,(width,mm) in panels.items():
        for ratio,density in zip([1,1.5,2,3],results[name]):
            assert abs(width/(density*ratio)-mm)<.1
            assert .8<density<24
    assert 'value="z13-qhd"' in html and 'value="g14-oled"' in html

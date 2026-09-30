"""Offline software recovery contracts; no hardware or strength acceptance."""
import asyncio
import ast
import contextlib
import importlib.util
import io
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import types
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'testing/dyno/ble-capture'))
from saris_h2 import protocol

fake_bleak = types.ModuleType('bleak')
class NoRealBleak:
    def __init__(self,*args,**kwargs):
        raise AssertionError('Real BLE construction is forbidden in offline tests')
fake_bleak.BleakClient = NoRealBleak
fake_bleak.BleakScanner = NoRealBleak
with patch.dict(sys.modules,{'bleak':fake_bleak}):
    from saris_h2 import control, ble_threaded

class FakeClient:
    def __init__(self): self.writes=[]; self.notifications=[]
    async def write_gatt_char(self,*args,**kwargs): self.writes.append((args,kwargs))
    async def start_notify(self,*args): self.notifications.append(args)

class ProtocolRecoveryTests(unittest.TestCase):
    def test_proprietary_known_packet_two_parameters(self):
        expected = b'\x00\x10\x02\x40\x1f\xce\xff\x00\x00\x00'
        self.assertEqual(protocol.build_resistance_cmd(protocol.ResistanceMode.MANUAL_SLOPE,8000,-50),expected)

    def test_headless_frame_is_not_a_release_claim(self):
        self.assertEqual(protocol.build_resistance_cmd(protocol.ResistanceMode.HEADLESS),bytes.fromhex('00100000000000000000'))
        self.assertIn('reject',protocol.ResistanceMode.__doc__)
        self.assertNotIn('free spin',protocol.build_resistance_cmd.__doc__)

    def test_cps_synthetic_optional_fields(self):
        packet=struct.pack('<HhHIHHH',0x34,125,64,1234,2048,34,1024)
        result=protocol.parse_cps(packet)
        self.assertEqual((result.power_w,result.acc_torque_nm,result.wheel_revs,result.wheel_event_time,result.crank_revs,result.crank_event_time),(125,2,1234,2048,34,1024))

    def test_short_cps_packets_reject(self):
        for packet in (b'',b'\x00',b'\x00\x00',struct.pack('<Hh',0x34,10)):
            with self.assertRaises(struct.error):protocol.parse_cps(packet)

    def test_release_brake_fails_without_transport_or_success_message(self):
        client=FakeClient(); output=io.StringIO()
        with contextlib.redirect_stdout(output),self.assertRaisesRegex(RuntimeError,'unsupported'):
            asyncio.run(control.release_brake(client))
        self.assertEqual(client.writes,[])
        self.assertEqual(client.notifications,[])
        self.assertEqual(output.getvalue(),'')

    def test_sim_request_uses_signed_grade_and_reports_request(self):
        client=FakeClient(); output=io.StringIO()
        with contextlib.redirect_stdout(output):asyncio.run(control.set_sim(client,80,-5))
        self.assertEqual(client.writes[0][0][1],bytes.fromhex('001002401fceff000000'))
        self.assertEqual(client.writes[0][1],{'response':False})
        self.assertIn('request',output.getvalue())
        self.assertNotIn('Released',output.getvalue())

    def test_notification_subscription_contract(self):
        client=FakeClient()
        asyncio.run(control.enable_resistance_notifications(client))
        self.assertEqual(client.notifications[0][0],protocol.SARIS_RESISTANCE_UUID)
        self.assertEqual(client.writes,[])

    def test_threaded_session_constructor_does_not_connect(self):
        session=ble_threaded.SarisH2Session('fixture-address',subscribe_proprietary=False)
        self.assertIsNone(session._client)
        self.assertIsNone(session._loop)
        self.assertEqual(session.events,[])
        with self.assertRaisesRegex(RuntimeError,'not started'):
            session.write_proprietary('fixture',b'\x00')

class ScreeningRecoveryTests(unittest.TestCase):
    def test_optional_screening_force_and_stress_scaling(self):
        from calc.gear_geometry import design_planetary_set
        from calc.tooth_stress import analyze_agma_stresses
        from calc.utils.data import PLA_PLUS
        gears=design_planetary_set(1.5,12,face_width_mm=10)
        a=analyze_agma_stresses(gears,PLA_PLUS,output_torque_nm=5)
        b=analyze_agma_stresses(gears,PLA_PLUS,output_torque_nm=10)
        self.assertAlmostEqual(b['tangential_force_n'],2*a['tangential_force_n'])
        self.assertAlmostEqual(b['bending_stress_sun_mpa'],2*a['bending_stress_sun_mpa'])
        self.assertAlmostEqual(b['contact_stress_sp_mpa'],math.sqrt(2)*a['contact_stress_sp_mpa'])
        self.assertEqual(a['allowable_source'],'yield-proxy')
        self.assertIn('not manufacturing',a['model_status'])

    def test_screening_print_does_not_claim_requirement_pass(self):
        from calc.gear_geometry import design_planetary_set
        from calc.tooth_stress import analyze_agma_stresses,print_agma_results
        from calc.utils.data import PLA_PLUS
        results=analyze_agma_stresses(design_planetary_set(1.5,12),PLA_PLUS)
        output=io.StringIO()
        with contextlib.redirect_stdout(output):print_agma_results(results)
        self.assertNotIn('PASS',output.getvalue())
        self.assertIn('not physical acceptance',output.getvalue())

    def test_thermistor_math_without_importing_or_launching_gui(self):
        # Execute only four inspected scalar functions; never import tkinter/matplotlib.
        path=ROOT/'testing/temperature-logger/thermistor_calc_gui.py'
        tree=ast.parse(path.read_text(encoding='utf-8'))
        names={'beta_t_from_r','beta_r_from_t','divider_voltage','adc_counts'}
        funcs=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in names]
        self.assertEqual(len(funcs),4)
        namespace={'math':math}
        exec(compile(ast.Module(body=funcs,type_ignores=[]),str(path),'exec'),namespace)
        self.assertAlmostEqual(namespace['beta_t_from_r'](10000,10000,3435,25),25)
        self.assertAlmostEqual(namespace['beta_r_from_t'](25,10000,3435,25),10000)
        self.assertAlmostEqual(namespace['divider_voltage'](10000,4.35,10000),2.175)
        self.assertAlmostEqual(namespace['adc_counts'](2.175,4.35,1023),511.5)

class OfflinePlotRecoveryTests(unittest.TestCase):
    def test_saved_data_plots_use_new_outputs_and_preserve_history(self):
        data=ROOT/'testing/mks-xdrive-mini/thermal_data'
        original=(data/'brake_hysteresis_overlay.png').read_bytes()
        env=dict(os.environ,MPLBACKEND='Agg',MPLCONFIGDIR=str(ROOT.parent/'mpl-cache'))
        with tempfile.TemporaryDirectory(dir=ROOT.parent) as folder:
            for name,outputs in [('compare_brake_tests.py',{'all_brake_tests_rpm.png','all_brake_tests_grid.png'}),
                                 ('plot_brake_hysteresis.py',{'brake_hysteresis_overlay.png'})]:
                destination=Path(folder)/name.replace('.py','')
                args=[sys.executable,str(ROOT/'testing/mks-xdrive-mini'/name),
                    '--input-dir',str(data),'--output-dir',str(destination)]
                result=subprocess.run(args,cwd=folder,env=env,capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertEqual({p.name for p in destination.iterdir()},outputs)
                for p in destination.iterdir():
                    self.assertTrue(p.read_bytes().startswith(b'\x89PNG\r\n\x1a\n'))
                again=subprocess.run(args,cwd=folder,env=env,capture_output=True,text=True)
                self.assertEqual(again.returncode,1)
                self.assertIn('never overwrite',again.stderr)
        self.assertEqual((data/'brake_hysteresis_overlay.png').read_bytes(),original)

# Execute each unchanged existing calculator case directly under unittest.
# Original file imports pytest only in its __main__ launcher. No fixtures or
# decorators are used, so this adapter preserves every actual assertion while
# using the already installed NumPy/SymPy interpreter without any installation.
spec=importlib.util.spec_from_file_location('existing_calc_tests',ROOT/'tests/test_calcs.py')
existing=importlib.util.module_from_spec(spec);spec.loader.exec_module(existing)
class ExistingCalculatorTests(unittest.TestCase): pass
for name,value in vars(existing).items():
    if isinstance(value,type) and name.startswith('Test'):
        for method in vars(value):
            if method.startswith('test_'):
                def adapter(self,klass=value,method_name=method):getattr(klass(),method_name)()
                setattr(ExistingCalculatorTests,'test_'+name+'_'+method,adapter)

if __name__=='__main__':unittest.main()

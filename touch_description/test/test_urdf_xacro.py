import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    description_file_path = os.path.join(
        get_package_share_directory("touch_description"), "urdf", "touch.urdf.xacro"
    )

    _, tmp_urdf_output_file = tempfile.mkstemp(suffix=".urdf")

    xacro_command = (
        f"{shutil.which('xacro')} {description_file_path} "
        f"use_mock_hardware:=true > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    try:
      xacro_process = subprocess.run(
          xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
      )
      assert xacro_process.returncode == 0, "xacro command failed"

      with open(tmp_urdf_output_file, "r", encoding="utf-8") as urdf_file:
          urdf_text = urdf_file.read()

      assert 'joint name="waist"' in urdf_text
      assert 'joint name="shoulder"' in urdf_text
      assert 'joint name="elbow"' in urdf_text
      assert 'joint name="yaw"' in urdf_text
      assert 'joint name="pitch"' in urdf_text
      assert 'joint name="roll"' in urdf_text
      assert 'link name="touch_tcp"' in urdf_text
      assert 'sensor name="tcp_pose"' in urdf_text
      assert 'gpio name="force"' in urdf_text
      assert "mock_components/GenericSystem" in urdf_text

      check_urdf_process = subprocess.run(
          check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
      )
      assert check_urdf_process.returncode == 0, "check_urdf failed"
    finally:
      os.remove(tmp_urdf_output_file)

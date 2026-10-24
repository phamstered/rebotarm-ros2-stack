from pathlib import Path
import os
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).parents[3]


def test_udp_profile_has_no_shared_memory_transport():
    xml = ET.parse(ROOT / 'src/joint_state_relay/config/fastdds_udp.xml')
    ns = {'p': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    assert [item.text for item in xml.findall('.//p:transport_descriptor/p:type', ns)] == ['UDPv4']
    assert xml.find('.//p:participant', ns).attrib['is_default_profile'] == 'true'
    assert xml.find('.//p:useBuiltinTransports', ns).text == 'false'


def test_startup_transport_respects_explicit_configuration_and_fails_if_missing(tmp_path):
    source = (ROOT / 'scripts/start_arm_teleop.sh').read_text()
    setup = source.split('# Process-scoped transport begin.', 1)[1].split('# Process-scoped transport end.', 1)[0]
    setup = '# Process-scoped transport begin.' + setup
    profile = tmp_path / 'src/joint_state_relay/config/fastdds_udp.xml'
    profile.parent.mkdir(parents=True)
    profile.write_text('fixture')
    env = dict(os.environ, WORKSPACE_DIR=str(tmp_path))
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    command = setup + '\nprintf "PROFILE=%s" "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"'
    result = subprocess.run(['bash', '-c', command], env=env, text=True, capture_output=True)
    assert result.returncode == 0 and f'PROFILE={profile}' in result.stdout
    for variable in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        explicit = subprocess.run(['bash', '-c', command], env={**env, variable: '/opt/custom.xml'}, text=True, capture_output=True)
        assert explicit.returncode == 0
        assert 'Using process-scoped' not in explicit.stdout
    profile.unlink()
    result = subprocess.run(['bash', '-c', command], env=env, text=True, capture_output=True)
    assert result.returncode != 0 and 'Missing Fast DDS transport profile' in result.stderr

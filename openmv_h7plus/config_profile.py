"""Optional field profile switch; keep formal runtime defaults in config.py."""

PROFILE = 'formal'
PROFILE_OVERRIDES = {
    'formal': {'TIMING_ENABLED': False, 'PROTOCOL_TRANSPORT': 'stdout'},
    'timing': {'TIMING_ENABLED': True, 'PROTOCOL_TRANSPORT': 'stdout'},
}

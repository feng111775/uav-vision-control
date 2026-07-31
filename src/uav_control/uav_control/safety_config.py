"""Pure validation for mission parameter safety combinations."""


def validate_safety_parameters(params):
    """Raise ValueError for combinations unsafe outside explicit SITL."""
    simulation = bool(params.get('simulation_mode', False))
    if not simulation and bool(params.get('auto_start_hover_test', False)):
        raise ValueError('auto_start_hover_test requires simulation_mode=true')
    if not simulation and bool(params.get('enable_auto_arm', False)):
        raise ValueError('enable_auto_arm requires simulation_mode=true')
    if not simulation and bool(params.get(
            'allow_sitl_heading_quality_bypass', False)):
        raise ValueError(
            'allow_sitl_heading_quality_bypass requires simulation_mode=true')
    if bool(params.get('communication_only', False)) and bool(
            params.get('auto_start_hover_test', False)):
        raise ValueError('communication_only cannot allow automatic start')
    return True

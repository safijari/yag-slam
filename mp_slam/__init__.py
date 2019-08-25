from mp_slam_cpp import ScanMatcherConfig

def print_config(config):
    for name in dir(config):
        if '_' == name[0]:
            continue
        print(f"{name}: {config.__getattribute__(name)}")


default_config = {
    "angle_variance_penalty": 1.0,
    "distance_variance_penalty": 0.5,
    "coarse_search_angle_offset": 0.3,
    "use_response_expansion": True,
    "range_threshold": 14,
    "minimum_angle_penalty": 1.5,
    "search_size": 0.5,
    "smear_deviation": 0.1,
}


def make_config(d):
    config = ScanMatcherConfig()
    config_params = default_config.copy()
    if d:
        config_params.update(d)

    for key, value in config_params.items():
        config.__setattr__(key, value)

    return config

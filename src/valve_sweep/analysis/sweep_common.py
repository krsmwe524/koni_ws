"""Common offline analysis for FESTO servo-valve voltage sweeps."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import numpy as np
import pandas as pd
import yaml


AI_TOPIC = '/ai1616llpe/voltage'
VOLTAGE_TOPIC = '/debug/sweep_voltage_V'
MEASURING_TOPIC = '/debug/sweep_is_measuring'
STEP_TOPIC = '/debug/sweep_step_index'
REQUIRED_TOPICS = (AI_TOPIC, VOLTAGE_TOPIC, MEASURING_TOPIC, STEP_TOPIC)


def load_config(path: str | Path = 'calibration.yaml') -> dict:
    """Load sensor calibration and model constants."""
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def find_mcap_files(bag_path: str | Path) -> list[Path]:
    """Return the MCAP file(s) contained in a rosbag directory."""
    path = Path(bag_path).expanduser()
    if path.is_file() and path.suffix == '.mcap':
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(f'Bag path does not exist: {path}')

    files = sorted(path.glob('*.mcap'))
    if not files:
        raise FileNotFoundError(f'No .mcap file found in: {path}')
    return files


def _merge_state(
    samples: pd.DataFrame,
    records: list[tuple[int, object]],
    column: str,
    tolerance_ns: int,
) -> pd.DataFrame:
    if not records:
        raise ValueError(f'Required sweep series is missing: {column}')
    state = pd.DataFrame(records, columns=['time_ns', column])
    state = state.sort_values('time_ns').drop_duplicates('time_ns', keep='last')
    return pd.merge_asof(
        samples.sort_values('time_ns'),
        state,
        on='time_ns',
        direction='backward',
        tolerance=tolerance_ns,
    )


def load_sweep_bag(bag_path: str | Path, config: dict) -> pd.DataFrame:
    """Read raw AI and sweep-state topics from an MCAP rosbag."""
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:
        raise ImportError(
            'Install the analysis dependencies with '
            '`python3 -m pip install -r requirements.txt`.'
        ) from exc

    ai_records: list[tuple[int, list[float]]] = []
    scalar_records: dict[str, list[tuple[int, object]]] = {
        VOLTAGE_TOPIC: [],
        MEASURING_TOPIC: [],
        STEP_TOPIC: [],
    }

    for mcap_path in find_mcap_files(bag_path):
        for record in read_ros2_messages(mcap_path, topics=REQUIRED_TOPICS):
            topic = record.channel.topic
            timestamp = int(record.log_time_ns)
            value = record.ros_msg.data
            if topic == AI_TOPIC:
                ai_records.append((timestamp, list(value)))
            elif topic in scalar_records:
                scalar_records[topic].append((timestamp, value))

    if not ai_records:
        raise ValueError(f'Required topic is missing: {AI_TOPIC}')

    channels = config['channels']
    max_channel = max(int(index) for index in channels.values())
    too_short = [len(values) for _, values in ai_records if len(values) <= max_channel]
    if too_short:
        raise ValueError(
            f'AI messages must contain channel {max_channel}; '
            f'found a message with {min(too_short)} elements.'
        )

    samples = pd.DataFrame(ai_records, columns=['time_ns', 'ai'])
    samples = samples.sort_values('time_ns').drop_duplicates('time_ns', keep='last')
    samples['pam_valve_voltage_v'] = samples['ai'].map(
        lambda values: float(values[channels['pam_valve_pressure']]))
    samples['supply_voltage_v'] = samples['ai'].map(
        lambda values: float(values[channels['supply_pressure']]))
    samples['flow_voltage_v'] = samples['ai'].map(
        lambda values: float(values[channels['flowmeter']]))
    samples = samples.drop(columns='ai')

    tolerance_ns = int(config['analysis']['sync_tolerance_s'] * 1e9)
    samples = _merge_state(
        samples,
        scalar_records[VOLTAGE_TOPIC],
        'command_voltage_v',
        tolerance_ns,
    )
    samples = _merge_state(
        samples,
        scalar_records[MEASURING_TOPIC],
        'is_measuring',
        tolerance_ns,
    )
    samples = _merge_state(
        samples,
        scalar_records[STEP_TOPIC],
        'step_index',
        tolerance_ns,
    )

    pressure = config['pressure']
    samples['supply_pressure_kpa_g'] = (
        samples['supply_voltage_v'] - pressure['supply_zero_v']
    ) * pressure['slope_kpa_per_v']
    samples['pam_valve_pressure_kpa_g'] = (
        samples['pam_valve_voltage_v'] - pressure['pam_valve_zero_v']
    ) * pressure['slope_kpa_per_v']

    flowmeter = config['flowmeter']
    samples['flow_l_min_anr'] = (
        samples['flow_voltage_v'] - flowmeter['zero_v']
    ) * flowmeter['slope_l_min_per_v']

    time_origin_ns = int(samples['time_ns'].min())
    samples['time_s'] = (samples['time_ns'] - time_origin_ns) / 1e9
    samples['step_index'] = samples['step_index'].astype('Int64')
    return samples


def select_measurement_tail(
    samples: pd.DataFrame,
    tail_duration_s: float,
) -> pd.DataFrame:
    """Select the final part of every valid measurement step."""
    measured = samples.loc[samples['is_measuring'] == True].copy()  # noqa: E712
    measured = measured.dropna(subset=['step_index', 'command_voltage_v'])
    if measured.empty:
        raise ValueError('No samples marked as measuring were found in the bag.')

    end_times = measured.groupby('step_index')['time_s'].transform('max')
    return measured.loc[measured['time_s'] >= end_times - tail_duration_s].copy()


def summarize_steps(samples: pd.DataFrame, tail_duration_s: float) -> pd.DataFrame:
    """Calculate mean and standard deviation for every voltage step."""
    tail = select_measurement_tail(samples, tail_duration_s)
    summary = tail.groupby('step_index', as_index=False).agg(
        sample_count=('time_s', 'size'),
        time_start_s=('time_s', 'min'),
        time_end_s=('time_s', 'max'),
        command_voltage_v=('command_voltage_v', 'mean'),
        flow_l_min_anr=('flow_l_min_anr', 'mean'),
        flow_std_l_min=('flow_l_min_anr', 'std'),
        supply_pressure_kpa_g=('supply_pressure_kpa_g', 'mean'),
        supply_pressure_std_kpa=('supply_pressure_kpa_g', 'std'),
        pam_valve_pressure_kpa_g=('pam_valve_pressure_kpa_g', 'mean'),
        pam_valve_pressure_std_kpa=('pam_valve_pressure_kpa_g', 'std'),
    )
    return summary.sort_values('step_index').reset_index(drop=True)


def flow_factor(pressure_ratio: np.ndarray, critical_ratio: float) -> np.ndarray:
    """Calculate phi(r) from the supplied effective-opening model."""
    ratio = np.asarray(pressure_ratio, dtype=float)
    factor = np.zeros_like(ratio)
    choked = ratio <= critical_ratio
    subsonic = (ratio > critical_ratio) & (ratio < 1.0)
    factor[choked] = 1.0
    factor[subsonic] = np.sqrt(
        np.maximum(
            0.0,
            1.0 - ((ratio[subsonic] - critical_ratio) /
                   (1.0 - critical_ratio)) ** 2,
        )
    )
    factor[~np.isfinite(ratio)] = np.nan
    return factor


def add_effective_area(
    summary: pd.DataFrame,
    direction: Literal['supply', 'exhaust'],
    config: dict,
) -> pd.DataFrame:
    """Add pressure ratio, mass flow, and effective area to a step summary."""
    result = summary.copy()
    air = config['air']
    atmospheric_kpa = float(air['atmospheric_pressure_kpa'])

    if direction == 'supply':
        result['upstream_pressure_kpa_g'] = result['supply_pressure_kpa_g']
        result['downstream_pressure_kpa_g'] = result[
            'pam_valve_pressure_kpa_g']
        result['flow_direction_ok'] = result['flow_l_min_anr'] >= 0.0
    elif direction == 'exhaust':
        result['upstream_pressure_kpa_g'] = result[
            'pam_valve_pressure_kpa_g']
        result['downstream_pressure_kpa_g'] = 0.0
        result['flow_direction_ok'] = result['flow_l_min_anr'] <= 0.0
    else:
        raise ValueError("direction must be 'supply' or 'exhaust'")

    result['upstream_pressure_kpa_abs'] = (
        result['upstream_pressure_kpa_g'] + atmospheric_kpa)
    result['downstream_pressure_kpa_abs'] = (
        result['downstream_pressure_kpa_g'] + atmospheric_kpa)
    result['pressure_ratio'] = (
        result['downstream_pressure_kpa_abs'] /
        result['upstream_pressure_kpa_abs'])

    gas_constant = float(air['specific_gas_constant_j_kg_k'])
    anr_temperature = float(air['anr_temperature_k'])
    anr_pressure_pa = float(air['anr_pressure_kpa']) * 1000.0
    anr_density = anr_pressure_pa / (gas_constant * anr_temperature)
    result['mass_flow_kg_s'] = (
        result['flow_l_min_anr'].abs() * 1e-3 / 60.0 * anr_density)

    kappa = float(air['heat_capacity_ratio'])
    upstream_temperature = float(air['upstream_temperature_k'])
    critical_ratio = float(config['model']['critical_pressure_ratio'])
    phi = flow_factor(result['pressure_ratio'].to_numpy(), critical_ratio)
    result['flow_factor_phi'] = phi

    pressure_pa = result['upstream_pressure_kpa_abs'] * 1000.0
    critical_term = (2.0 / (kappa + 1.0)) ** (
        (kappa + 1.0) / (kappa - 1.0))
    denominator = pressure_pa * np.sqrt(
        kappa / (gas_constant * upstream_temperature) * critical_term * phi)
    result['effective_area_m2'] = np.where(
        denominator > 0.0,
        result['mass_flow_kg_s'] / denominator,
        np.nan,
    )
    result['effective_area_mm2'] = result['effective_area_m2'] * 1e6

    full_scale = float(config['flowmeter']['full_scale_l_min'])
    result['flow_near_full_scale'] = (
        result['flow_l_min_anr'].abs() >= 0.98 * full_scale)
    result['valid_pressure_ratio'] = (
        (result['pressure_ratio'] >= 0.0) &
        (result['pressure_ratio'] < 1.0)
    )
    return result

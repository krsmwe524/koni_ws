# Valve sweep offline analysis

This directory contains separate notebooks for the valve supply and exhaust
characteristics. Both notebooks use `sweep_common.py` for MCAP loading, sensor
calibration, measurement-window selection, and effective-area calculation.

## Pressure mapping

| Direction | Upstream pressure | Downstream pressure |
|---|---|---|
| Supply | AI ch6, `supply_pressure` | AI ch1, `pam_valve_pressure` |
| Exhaust | AI ch1, `pam_valve_pressure` | Atmosphere |

All measured pressures are gauge pressures. Atmospheric pressure is added
before calculating the pressure ratio.

## Flow conversion

The TOKYO METER APM-L-200D output is treated as 1–5 V corresponding to
-200–200 L/min (ANR):

```text
Q_ANR = (V - 3) * 100 [L/min]
```

The manufacturer's ANR reference conditions are 20 °C and 101.325 kPa. The
analysis converts this volumetric flow to mass flow using the dry-air density
at those conditions.

## Setup

```bash
cd ~/koni_ws/src/valve_sweep/analysis
python3 -m pip install -r requirements.txt
jupyter lab
```

Open either `supply_analysis.ipynb` or `exhaust_analysis.ipynb`, set
`BAG_PATH`, and run all cells. `BAG_PATH` may point to the rosbag directory or
directly to its `.mcap` file.

By default, only the final 0.5 seconds of each interval marked by
`/debug/sweep_is_measuring` are averaged. Change `tail_duration_s` in
`calibration.yaml` if needed.

## Model

The notebooks implement the supplied effective-opening equation:

```text
S_e = m_dot / (P_up * sqrt(kappa/(R*T_up)
      * (2/(kappa+1))^((kappa+1)/(kappa-1))) * phi(r))
```

where `r = P_down/P_up` uses absolute pressure, and the provisional critical
pressure ratio is `h = 0.4`. Here, `phi(r)` is the elliptic approximation and
is applied directly to the choked mass-flow coefficient; it is not placed
inside another square root. Results are reported in square millimetres.

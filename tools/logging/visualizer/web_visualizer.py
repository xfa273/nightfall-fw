#!/usr/bin/env python3
import os
import sys
import hashlib
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from export_plotjuggler_csv import _build_output, _load_nightfall_csv


def _repo_root_from_this_file() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_logs_dir() -> Path:
    env_dir = os.environ.get("MICROMOUSE_LOG_DIR")
    if env_dir:
        return Path(env_dir).expanduser()
    return _repo_root_from_this_file() / "tools/logging/logs"


def _uploaded_logs_dir(logs_dir: Path) -> Path:
    return logs_dir / "uploaded"


def _coerce_8cols(df):
    import pandas as pd

    if df.shape[1] < 8:
        for i in range(df.shape[1], 8):
            df[i] = 0
    if df.shape[1] > 8:
        df = df.iloc[:, :8]

    cols = [
        "time_ms",
        "param1",
        "param2",
        "param3",
        "param4",
        "param5",
        "param6",
        "param7",
    ]
    df.columns = cols
    df = df.apply(pd.to_numeric, errors="coerce").fillna(0)
    return df


def _is_nightfall_trace_csv(path: Path) -> bool:
    cols = _extract_mm_columns(path)
    if cols is not None:
        return "timestamp_ms" in cols and "flags" in cols
    try:
        cols, _rows, _meta = _load_nightfall_csv(path)
    except Exception:
        return False
    return "timestamp_ms" in cols and "flags" in cols


def _list_csvs(logs_dir: Path) -> List[Tuple[str, Path, float, int]]:
    if not logs_dir.is_dir():
        return []

    out: List[Tuple[str, Path, float, int]] = []
    search_dirs = [logs_dir]
    uploaded_dir = _uploaded_logs_dir(logs_dir)
    if uploaded_dir.is_dir():
        search_dirs.append(uploaded_dir)

    for search_dir in search_dirs:
        for p in search_dir.iterdir():
            if not (p.is_file() and p.suffix.lower() == ".csv"):
                continue
            if p.name.endswith(".plotjuggler.csv"):
                continue
            if search_dir == logs_dir and p.name.startswith("uploaded_"):
                continue
            try:
                st = p.stat()
            except Exception:
                continue
            try:
                display_name = str(p.relative_to(logs_dir))
            except ValueError:
                display_name = p.name
            out.append((display_name, p, st.st_mtime, st.st_size))

    out.sort(key=lambda x: x[2], reverse=True)
    return out


def _uploaded_csv_path(logs_dir: Path, upload_name: str, content: bytes) -> Path:
    upload_dir = _uploaded_logs_dir(logs_dir)
    upload_dir.mkdir(parents=True, exist_ok=True)
    safe_name = Path(upload_name).name.replace("/", "_").replace("\\", "_")
    if not safe_name.lower().endswith(".csv"):
        safe_name = f"{safe_name}.csv"
    digest = hashlib.sha256(content).hexdigest()[:12]
    stem = Path(safe_name).stem
    return upload_dir / f"{stem}_{digest}.csv"


def _format_mtime(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def _extract_mm_columns(path: Path) -> Optional[List[str]]:
    try:
        with path.open("r", encoding="ascii", errors="ignore") as f:
            for _ in range(200):
                line = f.readline()
                if not line:
                    break
                s = line.strip()
                if s.startswith("#mm_columns="):
                    cols = [c.strip() for c in s[len("#mm_columns=") :].split(",")]
                    return cols

        tail_bytes = 64 * 1024
        with path.open("rb") as f2:
            try:
                f2.seek(0, os.SEEK_END)
                size = f2.tell()
                f2.seek(max(0, size - tail_bytes))
            except Exception:
                f2.seek(0)
            tail = f2.read().decode("ascii", errors="ignore")
        for line in reversed(tail.splitlines()):
            s = line.strip()
            if s.startswith("#mm_columns="):
                cols = [c.strip() for c in s[len("#mm_columns=") :].split(",")]
                return cols
    except Exception:
        pass
    return None


def _presets() -> Dict[str, List[str]]:
    return {
        "Default (param1..7)": [
            "param1",
            "param2",
            "param3",
            "param4",
            "param5",
            "param6",
            "param7",
        ],
        "Distance PID (旧GUIの既定)": [
            "target_distance",
            "real_distance",
            "PD_distance",
            "PI_distance",
            "PD_distance_d",
            "param6",
            "param7",
        ],
        "Omega PID / Motor (WALL_END_DERIV相当)": [
            "target_omega",
            "actual_omega",
            "p_term_omega",
            "i_term_omega",
            "d_term_omega",
            "motor_out_r",
            "motor_out_l",
        ],
        "Sensor (例)": [
            "ad_r",
            "ad_l",
            "ad_fr",
            "ad_fl",
            "distance",
            "param6",
            "param7",
        ],
    }


def _apply_column_names(df, names_7: List[str]):
    cols = ["time_ms"] + names_7
    df = df.copy()
    df.columns = cols
    return df


def _downsample(df, max_points: int):
    n = len(df)
    if max_points <= 0 or n <= max_points:
        return df
    step = max(1, n // max_points)
    return df.iloc[::step, :].reset_index(drop=True)


def _load_nightfall_trace_df(path: Path):
    import pandas as pd

    columns, rows, meta = _load_nightfall_csv(path)
    out_columns, out_rows = _build_output(columns, rows, meta, True)
    df = pd.DataFrame(out_rows, columns=out_columns)
    df = df.apply(pd.to_numeric, errors="coerce").fillna(0)
    return df, meta


def _build_plot(df, y_cols: List[str], mode: str):
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots

    x = df["time_ms"]
    palette = px.colors.qualitative.Plotly

    if mode == "Stacked (multiple graphs)":
        fig = make_subplots(rows=len(y_cols), cols=1, shared_xaxes=True, vertical_spacing=0.02)
        for i, col in enumerate(y_cols):
            color = palette[i % len(palette)]
            fig.add_trace(
                go.Scatter(x=x, y=df[col], mode="lines", name=col, line=dict(color=color)),
                row=i + 1,
                col=1,
            )
            fig.update_yaxes(title_text=col, row=i + 1, col=1)
        fig.update_xaxes(title_text="time_ms", row=len(y_cols), col=1)
        fig.update_layout(height=max(450, 180 * len(y_cols)), margin=dict(l=40, r=20, t=40, b=40))
        return fig

    fig = go.Figure()
    for i, col in enumerate(y_cols):
        color = palette[i % len(palette)]
        fig.add_trace(go.Scatter(x=x, y=df[col], mode="lines", name=col, line=dict(color=color)))
    fig.update_layout(
        height=650,
        margin=dict(l=40, r=20, t=40, b=40),
        xaxis_title="time_ms",
        yaxis_title="value",
        legend_title_text="params",
    )
    fig.update_xaxes(rangeslider_visible=False)
    return fig


def _build_nightfall_plot(df):
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go
    import plotly.express as px

    groups = [
        ("Distance / tune", "mm", ["distance_mm", "tune_ref", "tune_error"]),
        ("Velocity", "mm/s", ["target_velocity_mm_s", "real_velocity_mm_s", "velocity_error_mm_s"]),
        ("Motor", "PWM", ["motor_out_l", "motor_out_r", "motor_out_avg", "motor_out_diff"]),
        ("Angle", "deg", ["angle_deg", "target_angle_deg"]),
        ("Omega", "deg/s", ["target_omega_dps", "real_omega_dps", "omega_error_dps"]),
        ("Flags", "0/1", ["flag_motor_forward", "flag_motor_coast", "flag_motor_reverse", "flag_angle_target"]),
    ]

    available_groups = [(title, unit, cols) for title, unit, cols in groups if any(c in df.columns for c in cols)]
    fig = make_subplots(
        rows=len(available_groups),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.025,
        subplot_titles=[f"{title} [{unit}]" for title, unit, _ in available_groups],
    )

    palette = px.colors.qualitative.Plotly
    annotations = []
    for row, (title, unit, cols) in enumerate(available_groups, start=1):
        color_i = 0
        legend_lines = []
        for col in cols:
            if col not in df.columns:
                continue
            color = palette[color_i % len(palette)]
            fig.add_trace(
                go.Scatter(x=df["time_ms"], y=df[col], mode="lines", name=col, line=dict(color=color), showlegend=False),
                row=row,
                col=1,
            )
            legend_lines.append(f"<span style='color:{color}'>■</span> {col}")
            color_i += 1
        fig.update_yaxes(title_text=f"{title} [{unit}]", row=row, col=1)
        fig.update_xaxes(title_text="time_ms", showticklabels=True, row=row, col=1)
        y_domain = fig.layout[f"yaxis{row if row > 1 else ''}"].domain
        annotations.append(
            dict(
                x=1.01,
                y=sum(y_domain) * 0.5,
                xref="paper",
                yref="paper",
                text="<br>".join(legend_lines),
                showarrow=False,
                align="left",
                xanchor="left",
                yanchor="middle",
                font=dict(size=11),
            )
        )

    fig.update_layout(
        height=max(720, 240 * len(available_groups)),
        margin=dict(l=65, r=260, t=45, b=45),
        annotations=tuple(fig.layout.annotations) + tuple(annotations),
        hovermode="x unified",
    )
    return fig


def _render_export(fig, selected_csv: Path) -> int:
    import streamlit as st

    st.divider()
    st.subheader("Export")

    export_col1, export_col2 = st.columns([1, 2])
    with export_col1:
        export_name = st.text_input("PNG name", value=f"{selected_csv.stem}.png")
        save_dir = st.text_input("Save folder", value=str(selected_csv.parent))
        no_open = st.checkbox("Do not open after save", value=False)

        if st.button("Save PNG"):
            out_dir = Path(save_dir).expanduser()
            out_dir.mkdir(parents=True, exist_ok=True)
            out_path = out_dir / export_name
            try:
                png_bytes = fig.to_image(format="png")
            except Exception:
                st.error("PNG出力には kaleido が必要です。kaleido 1.x の場合は Google Chrome も必要です（plotly_get_chrome）。")
                return 0

            out_path.write_bytes(png_bytes)
            st.success(f"Saved: {out_path}")

            if (not no_open) and sys.platform == "darwin":
                try:
                    import subprocess

                    subprocess.run(["open", str(out_path)], check=False)
                except Exception:
                    pass

    with export_col2:
        try:
            png_bytes2 = fig.to_image(format="png")
            st.download_button("Download PNG", data=png_bytes2, file_name=f"{selected_csv.stem}.png", mime="image/png")
        except Exception:
            st.info("Downloadボタンも kaleido が必要です。kaleido 1.x の場合は Google Chrome も必要です（plotly_get_chrome）。")

    return 0


def _enable_auto_reload(seconds: int) -> None:
    import streamlit.components.v1 as components

    milliseconds = max(1, seconds) * 1000
    components.html(
        f"<script>setTimeout(function() {{ window.parent.location.reload(); }}, {milliseconds});</script>",
        height=0,
    )


def main() -> int:
    import streamlit as st

    st.set_page_config(page_title="Micromouse Log Visualizer (Web)", layout="wide")

    st.title("Micromouse Log Visualizer (Web)")

    default_logs_dir = _default_logs_dir()

    sidebar = st.sidebar
    sidebar.header("File")

    logs_dir_str = sidebar.text_input("Logs folder", value=str(default_logs_dir))
    logs_dir = Path(logs_dir_str).expanduser()

    if not logs_dir.is_dir():
        sidebar.error(f"Folder not found: {logs_dir}")

    if sidebar.button("Refresh log list"):
        st.cache_data.clear()
        st.rerun()

    auto_reload = sidebar.checkbox("Auto-refresh log list", value=False)
    if auto_reload:
        reload_seconds = sidebar.number_input("Refresh interval (sec)", min_value=1, max_value=60, value=3, step=1)
        _enable_auto_reload(int(reload_seconds))

    csv_entries = _list_csvs(logs_dir)

    default_csv = os.environ.get("MICROMOUSE_WEB_DEFAULT_CSV")
    default_csv_path: Optional[Path] = None
    if default_csv:
        p = Path(default_csv).expanduser()
        if p.exists():
            default_csv_path = p

    selected_csv: Optional[Path] = None

    if csv_entries:
        names = [f"{name}  ({_format_mtime(mtime)} / {size/1024:.1f}KB)" for name, _, mtime, size in csv_entries]
        paths = [p for _, p, _, _ in csv_entries]

        idx = 0
        if default_csv_path is not None:
            try:
                idx = paths.index(default_csv_path)
            except ValueError:
                idx = 0

        choice = sidebar.selectbox("CSV", options=list(range(len(names))), format_func=lambda i: names[i], index=idx)
        selected_csv = paths[choice]

    upload = sidebar.file_uploader("Upload CSV", type=["csv"])
    if upload is not None:
        tmp_dir = logs_dir if logs_dir.is_dir() else default_logs_dir
        content = upload.getbuffer().tobytes()
        uploaded_path = _uploaded_csv_path(tmp_dir, upload.name, content)
        if not uploaded_path.exists():
            uploaded_path.write_bytes(content)
        selected_csv = uploaded_path
        sidebar.success(f"Uploaded: {uploaded_path.relative_to(tmp_dir)}")

    if selected_csv is None:
        st.info("左のサイドバーでCSVを選択してください。")
        return 0

    st.caption(f"CSV: {selected_csv}")

    @st.cache_data(show_spinner=False)
    def _load_trace_cached(path_str: str, mtime: float):
        return _load_nightfall_trace_df(Path(path_str))

    @st.cache_data(show_spinner=False)
    def _load_csv_cached(path_str: str, mtime: float):
        import pandas as pd

        p = Path(path_str)
        mm_cols = _extract_mm_columns(p)
        df0 = pd.read_csv(path_str, header=None, comment="#")
        return _coerce_8cols(df0), mm_cols

    selected_mtime = selected_csv.stat().st_mtime
    x_min = sidebar.text_input("X min (ms)", value="")
    x_max = sidebar.text_input("X max (ms)", value="")
    max_points = sidebar.number_input("Max points (downsample)", min_value=1000, max_value=500000, value=80000, step=1000)

    if _is_nightfall_trace_csv(selected_csv):
        df_named, meta = _load_trace_cached(str(selected_csv), selected_mtime)
        df_view = df_named

        try:
            left = float(x_min) if x_min.strip() else None
            right = float(x_max) if x_max.strip() else None
            if left is not None:
                df_view = df_view[df_view["time_ms"] >= left]
            if right is not None:
                df_view = df_view[df_view["time_ms"] <= right]
        except ValueError:
            st.warning("X min / X max は数値で入力してください（空欄は自動）。")

        df_view = _downsample(df_view, int(max_points))
        fig = _build_nightfall_plot(df_view)

        duration_ms = float(df_named["time_ms"].max()) if "time_ms" in df_named.columns else 0.0
        info_cols = st.columns(4)
        info_cols[0].metric("Rows", f"{len(df_named)}")
        info_cols[1].metric("Duration", f"{duration_ms:.0f} ms")
        info_cols[2].metric("Format", meta.get("log_format", "nightfall_trace"))
        info_cols[3].metric("Git", meta.get("fw_git_sha", "-"))

        st.plotly_chart(fig, use_container_width=True)

        with st.expander("Preview (head)", expanded=False):
            st.dataframe(df_named.head(50))

        return _render_export(fig, selected_csv)

    df, mm_cols = _load_csv_cached(str(selected_csv), selected_mtime)

    sidebar.header("Columns")
    preset_names = _presets()
    if mm_cols is not None and len(mm_cols) == 8:
        preset_names = {"From log (#mm_columns)": mm_cols[1:], **preset_names}
    preset_idx = 0 if (mm_cols is not None and len(mm_cols) == 8) else min(1, max(0, len(preset_names) - 1))
    preset_key = sidebar.selectbox("Preset", options=list(preset_names.keys()), index=preset_idx)

    col_inputs: List[str] = []
    for i in range(7):
        col_inputs.append(sidebar.text_input(f"param{i+1}", value=preset_names[preset_key][i]))

    df_named = _apply_column_names(df, col_inputs)

    sidebar.header("Plot")
    available_cols = [c for c in df_named.columns if c != "time_ms"]
    default_cols = [c for c in available_cols if not c.startswith("unused")]
    if not default_cols:
        default_cols = available_cols
    selected_cols = sidebar.multiselect("Params", options=available_cols, default=default_cols)

    plot_mode = sidebar.radio("Mode", options=["Overlay (single graph)", "Stacked (multiple graphs)"], index=0)

    if not selected_cols:
        st.warning("表示するパラメータを選択してください。")
        return 0

    df_view = df_named

    try:
        left = float(x_min) if x_min.strip() else None
        right = float(x_max) if x_max.strip() else None
        if left is not None:
            df_view = df_view[df_view["time_ms"] >= left]
        if right is not None:
            df_view = df_view[df_view["time_ms"] <= right]
    except ValueError:
        st.warning("X min / X max は数値で入力してください（空欄は自動）。")

    df_view = _downsample(df_view, int(max_points))
    fig = _build_plot(df_view, selected_cols, plot_mode)

    st.plotly_chart(fig, use_container_width=True)

    with st.expander("Preview (head)", expanded=False):
        st.dataframe(df_named.head(50))

    return _render_export(fig, selected_csv)



if __name__ == "__main__":
    raise SystemExit(main())

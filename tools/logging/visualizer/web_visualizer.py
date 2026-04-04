#!/usr/bin/env python3
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _default_logs_dir() -> Path:
    env_dir = os.environ.get("MICROMOUSE_LOG_DIR")
    if env_dir:
        return Path(env_dir).expanduser()
    if sys.platform == "darwin":
        return Path.home() / "Documents/micromouse_logs"
    return Path.home() / "micromouse_logs"


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


def _list_csvs(logs_dir: Path) -> List[Tuple[str, Path, float, int]]:
    if not logs_dir.is_dir():
        return []

    out: List[Tuple[str, Path, float, int]] = []
    for p in logs_dir.iterdir():
        if not (p.is_file() and p.suffix.lower() == ".csv"):
            continue
        try:
            st = p.stat()
        except Exception:
            continue
        out.append((p.name, p, st.st_mtime, st.st_size))

    out.sort(key=lambda x: x[2], reverse=True)
    return out


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
        tmp_dir.mkdir(parents=True, exist_ok=True)
        uploaded_path = tmp_dir / f"uploaded_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{upload.name}"
        uploaded_path.write_bytes(upload.getbuffer())
        selected_csv = uploaded_path
        sidebar.success(f"Uploaded: {uploaded_path.name}")

    if selected_csv is None:
        st.info("左のサイドバーでCSVを選択してください。")
        return 0

    st.caption(f"CSV: {selected_csv}")

    @st.cache_data(show_spinner=False)
    def _load_csv_cached(path_str: str):
        import pandas as pd

        p = Path(path_str)
        mm_cols = _extract_mm_columns(p)
        df0 = pd.read_csv(path_str, header=None, comment="#")
        return _coerce_8cols(df0), mm_cols

    df, mm_cols = _load_csv_cached(str(selected_csv))

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

    x_min = sidebar.text_input("X min (ms)", value="")
    x_max = sidebar.text_input("X max (ms)", value="")

    max_points = sidebar.number_input("Max points (downsample)", min_value=1000, max_value=500000, value=80000, step=1000)

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


if __name__ == "__main__":
    raise SystemExit(main())

import streamlit as st
import pandas as pd
from sqlmodel import Session, select, func
import plotly.express as px
import plotly.graph_objects as go

# --- CHANGED: Import your models and engine ---
# Assuming models.py and database_setup_sqlmodel.py are in the same directory or PYTHONPATH
# Adjust the import path if needed.
try:
    from models import Experiment, Move # Import your SQLModel classes
    from database_setup import engine # Import the engine created by SQLModel
    SQLMODEL_AVAILABLE = True
except ImportError as e:
    st.error(f"Error importing SQLModel modules: {e}. Please ensure 'models.py' and 'database_setup_sqlmodel.py' are correctly set up and in the path.")
    SQLMODEL_AVAILABLE = False
    engine = None # Define engine to avoid NameError later, though the app will exit early

# --- CHANGED: Use the engine URL or a consistent path if needed for other libs ---
# DATABASE_PATH is not strictly needed for SQLModel queries anymore, but kept if other parts need it
# DATABASE_PATH = 'robot_performance.db' # Usually derived from engine.url

# --- UPDATED FUNCTION using SQLModel ---
# @st.cache_data # Caching with SQLModel objects can be tricky. Let's manage cache keys carefully or disable for now.
# Better to cache the final DataFrame result if needed, or handle caching at a higher level.
def load_experiment_names_sqlmodel():
    """Loads experiment names for selection using SQLModel."""
    if not SQLMODEL_AVAILABLE or engine is None:
        return pd.DataFrame() # Return empty DataFrame on import failure

    try:
        with Session(engine) as session:
            # Query Experiment objects
            statement = select(Experiment.id, Experiment.name).order_by(Experiment.start_time.desc())
            results = session.exec(statement).all()
            # Convert to DataFrame
            if results:
                df = pd.DataFrame(results, columns=['id', 'name'])
                return df
            else:
                return pd.DataFrame(columns=['id', 'name']) # Return empty DataFrame with correct columns
    except Exception as e:
        st.error(f"Error loading experiment names with SQLModel: {e}")
        return pd.DataFrame() # Return empty DataFrame on error

# --- UPDATED FUNCTION using SQLModel ---
# @st.cache_data
def load_experiment_data_sqlmodel(selected_experiment_ids):
    """Loads move data for selected experiments using SQLModel."""
    if not SQLMODEL_AVAILABLE or engine is None or not selected_experiment_ids:
        return pd.DataFrame()

    try:
        with Session(engine) as session:
            # Query Move objects joined with Experiment name, filtered by selected IDs
            statement = select(
                Move, # Select all columns from Move
                Experiment.name.label("experiment_name") # Select experiment name and alias it
            ).join(Experiment).where(Move.experiment_id.in_(selected_experiment_ids)).order_by(Move.timestamp)

            results = session.exec(statement).all()

            # Convert SQLModel instances to list of dictionaries for DataFrame
            # SQLModel instances have a .model_dump() method (Pydantic v2) or .dict() (older Pydantic/v1)
            moves_data = []
            for row in results:
                move_instance = row[0] # The Move instance
                exp_name = row[1]      # The experiment name string
                move_dict = move_instance.model_dump() # Convert Move instance to dict
                move_dict['experiment_name'] = exp_name # Add experiment name to the dict
                moves_data.append(move_dict)

            df = pd.DataFrame(moves_data)

            # --- Data Type Conversion (Crucial for Plotting) ---
            # Ensure numeric columns are floats, handle potential None/NULLs
            if not df.empty:
                numeric_columns = [
                    'move_number', 'success', 'total_time_seconds', 'planning_time_seconds',
                    'execution_time_seconds', 'placement_error_mm',
                    'min_collision_proximity_mm', 'retries'
                ]
                for col in numeric_columns:
                     if col in df.columns:
                         # pd.to_numeric with errors='coerce' converts non-numeric to NaN
                         df[col] = pd.to_numeric(df[col], errors='coerce')

                # Ensure boolean columns are boolean if needed (success is already int 0/1, often fine for plotting)
                # df['success'] = df['success'].astype(bool)

                # Convert timestamp string to datetime if needed (depends on how it's stored in DB)
                # df['timestamp'] = pd.to_datetime(df['timestamp'], errors='coerce') # Adjust format if necessary

            return df

    except Exception as e:
        st.error(f"Error loading experiment data with SQLModel: {e}")
        return pd.DataFrame() # Return empty DataFrame on error

def main():
    """Main Streamlit app logic."""
    # --- CHANGED: Check for SQLModel availability ---
    if not SQLMODEL_AVAILABLE:
        st.stop() # Stop the app if SQLModel setup failed

    st.set_page_config(page_title="Chess Robot Performance Dashboard (SQLModel)", layout="wide")
    st.title("♟️ Chess Robot Performance Dashboard (SQLModel)")

    # --- Sidebar for Controls ---
    st.sidebar.header("Controls")

    # --- CHANGED: Use SQLModel function ---
    # exp_df = load_experiment_names() # Old function
    exp_df = load_experiment_names_sqlmodel()

    if exp_df.empty:
        st.warning("No experiments found in the database. Run some simulations first!")
        return

    # Experiment Selection
    exp_options = dict(zip(exp_df['name'], exp_df['id']))
    selected_exp_names = st.sidebar.multiselect(
        "Select Experiments:",
        options=list(exp_options.keys()),
        default=list(exp_options.keys())[:1] # Default to first experiment
    )
    selected_exp_ids = [exp_options[name] for name in selected_exp_names]

    # Refresh Button
    if st.sidebar.button("Refresh Data"):
        # Note: st.cache_data decorator handles caching. Clearing cache might be needed if underlying DB changes frequently.
        # For SQLModel, it's often better to rely on the database's consistency or implement specific cache invalidation if needed.
        # Simple rerun is usually sufficient for Streamlit apps.
        st.rerun()

    # --- CHANGED: Use SQLModel function to load data ---
    # df = load_experiment_data(selected_exp_ids) # Old function
    df = load_experiment_data_sqlmodel(selected_exp_ids)

    if df.empty:
        st.info("Select one or more experiments to view data.")
        return

    # --- Main Dashboard Content ---
    # (The rest of the plotting logic remains LARGELY the same, as it operates on the Pandas DataFrame `df`)
    # The key benefit of SQLModel is in the data fetching layer above.

    tab1, tab2, tab3, tab4, tab5, tab6 = st.tabs([
        "Overview", "Trends", "Distributions", "Correlations", "Failures", "Comparisons"
    ])

    # --- Tab 1: Overview / Summary ---
    with tab1:
        st.header("Overview")
        # Use the number of selected experiments for columns
        cols = st.columns(len(selected_exp_names)) if selected_exp_names else [st] # Fallback if somehow none selected but code runs

        for i, exp_name in enumerate(selected_exp_names):
            exp_id = exp_options[exp_name]
            # Filter DataFrame for the specific experiment
            exp_df_filtered = df[df['experiment_id'] == exp_id]

            total_moves = len(exp_df_filtered)
            successful_moves = exp_df_filtered['success'].sum() # Sum of 1s (True) in the success column
            success_rate = (successful_moves / total_moves * 100) if total_moves > 0 else 0
            # Use .mean() for average times, which automatically ignores NaN/None
            avg_total_time = exp_df_filtered['total_time_seconds'].mean()
            avg_planning_time = exp_df_filtered['planning_time_seconds'].mean()
            avg_execution_time = exp_df_filtered['execution_time_seconds'].mean()

            # Display metrics in the column
            with cols[i]:
                st.subheader(exp_name)
                st.metric("Success Rate", f"{success_rate:.1f}%")
                st.metric("Avg Total Time", f"{avg_total_time:.2f}s")
                st.metric("Avg Planning Time", f"{avg_planning_time:.2f}s")
                st.metric("Avg Execution Time", f"{avg_execution_time:.2f}s")
                st.write(f"Total Moves: {total_moves}")

        # Overall Success Rate Comparison (Bar Chart) - Only if multiple experiments
        if len(selected_exp_names) > 1:
            st.subheader("Success Rate Comparison")
            success_rates = []
            for exp_name in selected_exp_names:
                exp_id = exp_options[exp_name]
                exp_df_f = df[df['experiment_id'] == exp_id]
                total = len(exp_df_f)
                success = exp_df_f['success'].sum()
                rate = (success / total * 100) if total > 0 else 0
                success_rates.append({'Experiment': exp_name, 'Success Rate (%)': rate})

            sr_df = pd.DataFrame(success_rates)
            if not sr_df.empty:
                fig_sr = px.bar(sr_df, x='Experiment', y='Success Rate (%)', text_auto='.1f')
                fig_sr.update_traces(textposition='outside')
                # Use container width for responsive charts
                st.plotly_chart(fig_sr, use_container_width=True)


    # --- Tab 2: Trends Over Time ---
    with tab2:
        st.header("Trends Over Time")

        # Success Rate vs Move Number (per experiment)
        st.subheader("Rolling Success Rate")
        # Slider for window size
        window_size = st.slider("Rolling Window Size", min_value=5, max_value=50, value=10, key='trend_window')
        trend_fig = go.Figure()
        for exp_name in selected_exp_names:
            exp_id = exp_options[exp_name]
            # Sort by move_number to ensure correct rolling window calculation
            exp_df_sorted = df[df['experiment_id'] == exp_id].sort_values('move_number').copy()
            if not exp_df_sorted.empty:
                # Calculate rolling success rate
                exp_df_sorted['rolling_success_rate'] = exp_df_sorted['success'].rolling(window=window_size, min_periods=1).mean() * 100
                trend_fig.add_trace(go.Scatter(
                    x=exp_df_sorted['move_number'],
                    y=exp_df_sorted['rolling_success_rate'],
                    mode='lines+markers',
                    name=exp_name,
                    hovertemplate='Move: %{x}<br>Rolling Success Rate: %{y:.1f}%<extra></extra>'
                ))
        trend_fig.update_layout(
            title=f"Rolling Success Rate (Window: {window_size})",
            xaxis_title="Move Number",
            yaxis_title="Success Rate (%)",
            yaxis_range=[0, 105] # Set Y-axis range for percentage
        )
        st.plotly_chart(trend_fig, use_container_width=True)

        # Time vs Move Number (Individual Line Charts for each time metric)
        st.subheader("Time Metrics vs Move Number")
        # Define columns and labels for time metrics
        time_cols = ['total_time_seconds', 'planning_time_seconds', 'execution_time_seconds']
        time_labels = ['Total Time (s)', 'Planning Time (s)', 'Execution Time (s)']
        # Create a line chart for each time metric
        for col, label in zip(time_cols, time_labels):
            time_fig = go.Figure()
            for exp_name in selected_exp_names:
                exp_id = exp_options[exp_name]
                # Filter, sort, and remove rows with NaN in the specific time column
                exp_df_sorted = df[(df['experiment_id'] == exp_id) & (df[col].notnull())].sort_values('move_number')
                if not exp_df_sorted.empty:
                    time_fig.add_trace(go.Scatter(
                        x=exp_df_sorted['move_number'],
                        y=exp_df_sorted[col],
                        mode='lines+markers', # Show lines and markers
                        name=exp_name,
                        hovertemplate='Move: %{x}<br>' + label + ': %{y:.2f}s<extra></extra>' # Custom hover template
                    ))
            time_fig.update_layout(title=label, xaxis_title="Move Number", yaxis_title=label)
            st.plotly_chart(time_fig, use_container_width=True) # Use container width


    # --- Tab 3: Distributions ---
    with tab3:
        st.header("Distributions")

        # Create two columns for side-by-side histograms
        hist_col1, hist_col2 = st.columns(2)

        # --- Histogram 1: Total Move Time (Successful Moves) ---
        with hist_col1:
            st.subheader("Total Move Time (Successful)")
            # Filter for successful moves only
            successful_df = df[df['success'] == True]
            if not successful_df.empty:
                # Create histogram using Plotly Express
                fig_hist_total = px.histogram(successful_df, x='total_time_seconds', nbins=30,
                                               title="Distribution of Total Move Times (Successful Moves)")
                st.plotly_chart(fig_hist_total, use_container_width=True) # Use container width
            else:
                st.info("No successful moves to display for this distribution.")

        # --- Histogram 2: Planning Time (All Moves) ---
        with hist_col2:
            st.subheader("Planning Time")
            # Check if there's any data in the planning_time_seconds column (excluding NaN)
            if not df['planning_time_seconds'].dropna().empty:
                fig_hist_plan = px.histogram(df, x='planning_time_seconds', nbins=30,
                                             title="Distribution of Planning Times")
                st.plotly_chart(fig_hist_plan, use_container_width=True)
            else:
                st.info("No planning times recorded.")

        # --- Histogram 3 & 4: Placement Error & Collision Proximity ---
        hist_col3, hist_col4 = st.columns(2)

        with hist_col3:
            st.subheader("Placement Error (mm)")
            # Filter for rows with placement error data
            error_df = df[df['placement_error_mm'].notnull()]
            if not error_df.empty:
                fig_hist_error = px.histogram(error_df, x='placement_error_mm', nbins=30,
                                               title="Distribution of Placement Errors")
                st.plotly_chart(fig_hist_error, use_container_width=True)
            else:
                st.info("No placement errors recorded.")

        with hist_col4:
            st.subheader("Min Collision Proximity (mm)")
            # Filter for rows with collision proximity data
            prox_df = df[df['min_collision_proximity_mm'].notnull()]
            if not prox_df.empty:
                fig_hist_prox = px.histogram(prox_df, x='min_collision_proximity_mm', nbins=30,
                                             title="Distribution of Min Collision Proximities")
                st.plotly_chart(fig_hist_prox, use_container_width=True)
            else:
                st.info("No collision proximity data recorded.")


    # --- Tab 4: Correlations ---
    with tab4:
        st.header("Correlations")

        # Create two columns for side-by-side scatter plots
        corr_col1, corr_col2 = st.columns(2)

        # --- Scatter 1: Placement Error vs Planning Time ---
        with corr_col1:
            st.subheader("Error vs Planning Time")
            # Drop rows with NaN in either placement_error_mm or planning_time_seconds
            error_time_df = df.dropna(subset=['placement_error_mm', 'planning_time_seconds'])
            if not error_time_df.empty:
                # Create scatter plot
                fig_scatter_et = px.scatter(error_time_df, x='planning_time_seconds', y='placement_error_mm',
                                            hover_data=['experiment_name', 'move_number', 'source_square', 'target_square'], # Add details on hover
                                            title="Placement Error vs Planning Time")
                st.plotly_chart(fig_scatter_et, use_container_width=True)
            else:
                st.info("Insufficient data for Error vs Planning Time scatter plot.")

        # --- Scatter 2: Collision Proximity vs Placement Error ---
        with corr_col2:
            st.subheader("Proximity vs Error")
            # Drop rows with NaN in either min_collision_proximity_mm or placement_error_mm
            prox_error_df = df.dropna(subset=['min_collision_proximity_mm', 'placement_error_mm'])
            if not prox_error_df.empty:
                fig_scatter_pe = px.scatter(prox_error_df, x='min_collision_proximity_mm', y='placement_error_mm',
                                             hover_data=['experiment_name', 'move_number', 'source_square', 'target_square'],
                                             title="Min Collision Proximity vs Placement Error")
                st.plotly_chart(fig_scatter_pe, use_container_width=True)
            else:
                st.info("Insufficient data for Proximity vs Error scatter plot.")

        # Add more correlation plots as needed (e.g., Path Length proxy vs Time)


    # --- Tab 5: Failures ---
    with tab5:
        st.header("Failures")

        # Filter the main DataFrame to only include failed moves (success == False)
        failed_df = df[df['success'] == False].copy() # Use .copy() to avoid SettingWithCopyWarning later if needed

        # --- Failure Type Counts ---
        st.subheader("Failure Counts by Type")
        if not failed_df.empty:
            # Group by experiment name and failure type, then count occurrences
            failure_summary = failed_df.groupby(['experiment_name', 'failure_type']).size().reset_index(name='count')
            # Handle potential None/NaN in failure_type if they exist
            failure_summary = failure_summary.dropna(subset=['failure_type'])

            if not failure_summary.empty:
                # Pivot the data for easier plotting with Plotly Express
                # This reshapes the data so each failure type is a column
                failure_pivot = failure_summary.pivot(index='experiment_name', columns='failure_type', values='count').fillna(0)
                # Melt (or unpivot) the pivoted data back into a format suitable for bar charts
                # 'value_name' specifies the new column name for the counts
                failure_melted = failure_pivot.reset_index().melt(id_vars='experiment_name', var_name='Failure Type', value_name='Count')

                # Create a stacked bar chart
                fig_fail_bar = px.bar(failure_melted, x='experiment_name', y='Count', color='Failure Type',
                                      title="Failure Counts by Type and Experiment")
                st.plotly_chart(fig_fail_bar, use_container_width=True)
            else:
                 st.info("No specific failure types recorded for failures.")
        else:
            st.info("No failed moves recorded for the selected experiments.")

        # --- Failure Details Table ---
        st.subheader("Failure Details")
        # Display the filtered DataFrame containing only failures
        # Select relevant columns for the table
        if not failed_df.empty:
            detail_columns = ['experiment_name', 'move_number', 'source_square', 'target_square', 'piece_type', 'failure_type', 'timestamp']
            # Ensure only columns present in the DataFrame are selected
            existing_columns = [col for col in detail_columns if col in failed_df.columns]
            if existing_columns:
                st.dataframe(failed_df[existing_columns].reset_index(drop=True)) # Reset index for display
            else:
                 st.info("No relevant failure detail columns found in data.")
        else:
             st.info("No failure details to display.")


    # --- Tab 6: Comparisons ---
    with tab6:
        st.header("Comparisons")

        # Explanation text
        st.info("This tab primarily uses overlaid plots from other tabs for comparison. "
                "Select multiple experiments in the sidebar to see comparisons.")

        # --- Example: Overlaid Histograms ---
        st.subheader("Overlaid Placement Error Distributions")
        # Filter for rows with placement error data
        error_df = df[df['placement_error_mm'].notnull()]
        if not error_df.empty:
            # Create an overlaid histogram, coloring by experiment_name
            fig_compare_error = px.histogram(error_df, x='placement_error_mm', color='experiment_name', nbins=30,
                                             title="Placement Error Distribution Comparison",
                                             opacity=0.7) # Add opacity for better visualization of overlaps
            st.plotly_chart(fig_compare_error, use_container_width=True)
        else:
            st.info("No placement errors recorded for comparison.")

        # You can add more specific comparison plots here if desired,
        # such as comparing average times side-by-side using bar charts generated from grouped data.

if __name__ == "__main__":
    main()
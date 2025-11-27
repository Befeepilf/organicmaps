package app.organicmaps.stats;

import android.app.Dialog;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Spinner;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.DialogFragment;
import androidx.fragment.app.FragmentManager;

import com.google.android.material.dialog.MaterialAlertDialogBuilder;

import app.organicmaps.R;
import app.organicmaps.sdk.stats.ExploreStats;

public class ExploreStatsDialogFragment extends DialogFragment
{
  public static void show(@NonNull FragmentManager fm)
  {
    new ExploreStatsDialogFragment().show(fm, "explore_stats_dialog");
  }

  @NonNull
  @Override
  public Dialog onCreateDialog(@Nullable Bundle savedInstanceState)
  {
    View view = LayoutInflater.from(requireContext()).inflate(R.layout.dialog_explore_stats, null);
    Spinner spinner = view.findViewById(R.id.region_spinner);
    android.widget.TextView tv = view.findViewById(R.id.tv_weeks);

    ExploreStats.Payload payload = ExploreStats.nativeGetAll();
    java.util.List<String> ids = new java.util.ArrayList<>();
    java.util.List<String> names = new java.util.ArrayList<>();
    names.add(getString(R.string.all_regions));
    ids.add("");
    if (payload != null && payload.regions != null)
    {
      for (ExploreStats.Region r : payload.regions)
      {
        ids.add(r.id);
        names.add(r.name);
      }
    }
    ArrayAdapter<String> adapter = new ArrayAdapter<>(requireContext(), android.R.layout.simple_spinner_dropdown_item, names);
    spinner.setAdapter(adapter);

    java.util.function.Consumer<ExploreStats.Payload> render = (pl) -> {
      StringBuilder sb = new StringBuilder();
      if (pl != null && pl.weeks != null)
      {
        for (ExploreStats.StatWeek w : pl.weeks)
        {
          java.time.Instant inst = java.time.Instant.ofEpochSecond(w.weekStartSec);
          java.time.LocalDate date = java.time.LocalDateTime.ofInstant(inst, java.time.ZoneId.systemDefault()).toLocalDate();
          sb.append(date.toString()).append(": ").append(w.exploredPixels).append("\n");
        }
      }
      tv.setText(sb.toString());
    };

    render.accept(payload);

    spinner.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener()
    {
      @Override public void onItemSelected(android.widget.AdapterView<?> parent, View v, int position, long id)
      {
        String regionId = ids.get(position);
        render.accept(ExploreStats.nativeGetForRegion(regionId));
      }
      @Override public void onNothingSelected(android.widget.AdapterView<?> parent) {}
    });

    return new MaterialAlertDialogBuilder(requireContext(), R.style.MwmTheme_AlertDialog)
        .setTitle(R.string.explore_stats)
        .setView(view)
        .setPositiveButton(R.string.ok, null)
        .create();
  }
}
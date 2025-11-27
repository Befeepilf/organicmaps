package app.organicmaps.settings;

import android.app.Dialog;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.EditText;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.DialogFragment;
import androidx.fragment.app.FragmentManager;

import com.google.android.material.dialog.MaterialAlertDialogBuilder;
import com.google.android.material.switchmaterial.SwitchMaterial;

import app.organicmaps.sdk.Framework;
import app.organicmaps.sdk.friends.Friends;
import app.organicmaps.R;

public class MyAccountDialogFragment extends DialogFragment
{
  public static void show(@NonNull FragmentManager fm)
  {
    new MyAccountDialogFragment().show(fm, "my_account_dialog");
  }

  @NonNull
  @Override
  public Dialog onCreateDialog(@Nullable Bundle savedInstanceState)
  {
    LayoutInflater inflater = LayoutInflater.from(requireContext());
    View view = inflater.inflate(R.layout.dialog_my_account, null);

    final SwitchMaterial shareSwitch = view.findViewById(R.id.share_switch);
    shareSwitch.setChecked(Framework.nativeGetExploreSharingEnabled());
    final EditText username = view.findViewById(R.id.username_edit);
    String current = Framework.nativeHasUsername() ? Framework.nativeGetUsername() : null;
    if (current != null)
      username.setText(current);

    // Friends section
    final View friendsSection = view.findViewById(R.id.friends_section);
    final EditText inputSearch = view.findViewById(R.id.input_search);
    final View btnSearch = view.findViewById(R.id.btn_search);
    final android.widget.TextView tvAccepted = view.findViewById(R.id.tv_accepted);
    final android.widget.TextView tvIncoming = view.findViewById(R.id.tv_incoming);
    final android.widget.TextView tvOutgoing = view.findViewById(R.id.tv_outgoing);

    friendsSection.setEnabled(shareSwitch.isChecked());
    friendsSection.setAlpha(shareSwitch.isChecked() ? 1f : 0.5f);
    shareSwitch.setOnCheckedChangeListener((buttonView, isChecked) -> {
      friendsSection.setEnabled(isChecked);
      friendsSection.setAlpha(isChecked ? 1f : 0.5f);
    });

    btnSearch.setOnClickListener(v -> {
      String q = inputSearch.getText().toString().trim();
      if (q.isEmpty()) return;
      Friends.Friend[] results = Friends.nativeSearchByUsername(q);
      StringBuilder sb = new StringBuilder();
      if (results != null)
        for (Friends.Friend f : results)
          sb.append(f.username).append(" (id:").append(f.userId).append(")\n");
      tvOutgoing.setText(sb.toString());
    });

    Friends.FriendsPayload lists = Friends.nativeGetLists();
    if (lists != null)
    {
      StringBuilder a = new StringBuilder();
      if (lists.accepted != null)
        for (Friends.Friend f : lists.accepted) a.append(f.username).append("\n");
      tvAccepted.setText(a.toString());

      StringBuilder i = new StringBuilder();
      if (lists.incoming != null)
        for (Friends.Friend f : lists.incoming) i.append(f.username).append("\n");
      tvIncoming.setText(i.toString());

      StringBuilder o = new StringBuilder();
      if (lists.outgoing != null)
        for (Friends.Friend f : lists.outgoing) o.append(f.username).append("\n");
      tvOutgoing.setText(o.toString());
    }

    return new MaterialAlertDialogBuilder(requireContext(), R.style.MwmTheme_AlertDialog)
        .setTitle(R.string.my_account)
        .setView(view)
        .setPositiveButton(R.string.save, (d, w) -> {
          Framework.nativeSetExploreSharingEnabled(shareSwitch.isChecked());
          String name = username.getText().toString().trim();
          if (!name.isEmpty())
            Framework.nativeSetUsername(name);
        })
        .setNegativeButton(R.string.cancel, null)
        .create();
  }
}
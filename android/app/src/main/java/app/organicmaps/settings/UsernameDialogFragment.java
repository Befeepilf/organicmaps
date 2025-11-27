package app.organicmaps.settings;

import android.app.Dialog;
import android.os.Bundle;
import android.text.InputFilter;
import android.text.InputType;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.DialogFragment;

import com.google.android.material.dialog.MaterialAlertDialogBuilder;

import app.organicmaps.sdk.Framework;
import app.organicmaps.R;

public class UsernameDialogFragment extends DialogFragment
{
  public interface Listener { void onUsernameEntered(@NonNull String username); }

  @Nullable private Listener mListener;

  public static void show(@NonNull SettingsPrefsFragment host, @NonNull Listener listener)
  {
    UsernameDialogFragment f = new UsernameDialogFragment();
    f.mListener = listener;
    f.show(host.getParentFragmentManager(), "username_dialog");
  }

  @NonNull
  @Override
  public Dialog onCreateDialog(@Nullable Bundle savedInstanceState)
  {
    final EditText input = new EditText(requireContext());
    input.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_VISIBLE_PASSWORD);
    input.setImeOptions(EditorInfo.IME_FLAG_NO_PERSONALIZED_LEARNING);
    input.setFilters(new InputFilter[]{ new InputFilter.LengthFilter(20) });
    String current = Framework.nativeHasUsername() ? Framework.nativeGetUsername() : null;
    if (current != null)
      input.setText(current);

    return new MaterialAlertDialogBuilder(requireContext(), R.style.MwmTheme_AlertDialog)
        .setTitle(R.string.pref_explore_username_title)
        .setMessage(R.string.pref_explore_username_hint)
        .setView(input)
        .setPositiveButton(R.string.save, (d, w) -> {
          String value = input.getText().toString().trim();
          if (mListener != null)
            mListener.onUsernameEntered(value);
        })
        .setNegativeButton(R.string.cancel, null)
        .create();
  }
}
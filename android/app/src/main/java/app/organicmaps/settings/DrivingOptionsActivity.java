package app.organicmaps.settings;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;

import androidx.activity.result.ActivityResultLauncher;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import app.organicmaps.base.BaseMwmFragmentActivity;
import app.organicmaps.sdk.Router;

public class DrivingOptionsActivity extends BaseMwmFragmentActivity
{
  private static final String EXTRA_ROUTER_TYPE = "router_type";

  @Override
  protected Class<? extends Fragment> getFragmentClass()
  {
    return DrivingOptionsFragment.class;
  }

  @Override
  protected void onResume()
  {
    super.onResume();

    Fragment fragment = getSupportFragmentManager().findFragmentById(getFragmentContentResId());
    if (fragment instanceof DrivingOptionsFragment)
    {
      Router routerType = (Router) getIntent().getSerializableExtra(EXTRA_ROUTER_TYPE);
      ((DrivingOptionsFragment) fragment).setRouterType(routerType);
    }
  }

  public static void start(@NonNull Activity activity, ActivityResultLauncher<Intent> startDrivingOptionsForResult)
  {
    start(activity, startDrivingOptionsForResult, null);
  }

  public static void start(@NonNull Activity activity, ActivityResultLauncher<Intent> startDrivingOptionsForResult,
                          Router routerType)
  {
    Intent intent = new Intent(activity, DrivingOptionsActivity.class);
    if (routerType != null)
    {
      intent.putExtra(EXTRA_ROUTER_TYPE, routerType);
    }
    startDrivingOptionsForResult.launch(intent);
  }
}

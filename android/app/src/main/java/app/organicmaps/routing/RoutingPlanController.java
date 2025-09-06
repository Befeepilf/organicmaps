package app.organicmaps.routing;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import androidx.appcompat.widget.SwitchCompat;

import androidx.activity.result.ActivityResultLauncher;
import androidx.annotation.DrawableRes;
import androidx.annotation.IdRes;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;
import app.organicmaps.Framework;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.Router;
import app.organicmaps.sdk.routing.RoutingInfo;
import app.organicmaps.sdk.routing.RoutingOptions;
import app.organicmaps.sdk.routing.TransitRouteInfo;
import app.organicmaps.settings.RoadType;
import app.organicmaps.settings.TrailRoutingSettings;
import app.organicmaps.settings.DrivingOptionsActivity;
import app.organicmaps.util.UiUtils;
import app.organicmaps.util.WindowInsetUtils.PaddingInsetsListener;
import app.organicmaps.widget.RoutingToolbarButton;
import app.organicmaps.widget.ToolbarController;
import app.organicmaps.widget.WheelProgressView;

public class RoutingPlanController extends ToolbarController
{
  private static final String TAG = RoutingPlanController.class.getSimpleName();
  private static final String BUNDLE_HAS_DRIVING_OPTIONS_VIEW = "has_driving_options_view";

  private final View mFrame;
  @NonNull
  private final RoutingPlanInplaceController.RoutingPlanListener mRoutingPlanListener;
  private final RadioGroup mRouterTypes;
  @NonNull
  private final WheelProgressView mProgressVehicle;
  @NonNull
  private final WheelProgressView mProgressPedestrian;
  @NonNull
  private final WheelProgressView mProgressTransit;
  @NonNull
  private final WheelProgressView mProgressBicycle;
  @NonNull
  private final WheelProgressView mProgressRuler;

//  @NonNull
//  private final WheelProgressView mProgressTaxi;

  @NonNull
  private final RoutingBottomMenuController mRoutingBottomMenuController;

  int mFrameHeight;
  final int mAnimToggle;

  @NonNull
  private final View mRoutingOptionsContainer;

  @NonNull
  private final View mRoutingOptionsGearBtn;

  @NonNull
  private final View mRoutingOptionsHeader;

  @NonNull
  private final View mCarRoutingOptionsContainer;

  @NonNull
  private final View mTrailOptionsContainer;

  private void setupRouterButton(@IdRes int buttonId, final @DrawableRes int iconRes, View.OnClickListener clickListener)
  {
    CompoundButton.OnCheckedChangeListener listener = (buttonView, isChecked) -> {
      RoutingToolbarButton button = (RoutingToolbarButton) buttonView;
      button.setIcon(iconRes);
      if (isChecked)
        button.activate();
      else
        button.deactivate();
    };

    RoutingToolbarButton rb = mRouterTypes.findViewById(buttonId);
    listener.onCheckedChanged(rb, false);
    rb.setOnCheckedChangeListener(listener);
    rb.setOnClickListener(clickListener);
  }

  RoutingPlanController(View root, Activity activity,
                        ActivityResultLauncher<Intent> startDrivingOptionsForResult,
                        @NonNull RoutingPlanInplaceController.RoutingPlanListener routingPlanListener,
                        @Nullable RoutingBottomMenuListener listener)
  {
    super(root, activity);
    mFrame = root;
    mRoutingPlanListener = routingPlanListener;

    mRouterTypes = getToolbar().findViewById(R.id.route_type);

    setupRouterButtons();

    View progressFrame = getToolbar().findViewById(R.id.progress_frame);
    mProgressVehicle = progressFrame.findViewById(R.id.progress_vehicle);
    mProgressPedestrian = progressFrame.findViewById(R.id.progress_pedestrian);
    mProgressTransit = progressFrame.findViewById(R.id.progress_transit);
    mProgressBicycle = progressFrame.findViewById(R.id.progress_bicycle);
    mProgressRuler = progressFrame.findViewById(R.id.progress_ruler);
//    mProgressTaxi = (WheelProgressView) progressFrame.findViewById(R.id.progress_taxi);

    mRoutingBottomMenuController = RoutingBottomMenuController.newInstance(requireActivity(), mFrame, listener);

    // Initialize new routing options UI
    mRoutingOptionsContainer = mFrame.findViewById(R.id.routing_options_container);
    mRoutingOptionsGearBtn = mFrame.findViewById(R.id.routing_options_gear_btn);
    mRoutingOptionsHeader = mFrame.findViewById(R.id.routing_options_header);
    mCarRoutingOptionsContainer = mFrame.findViewById(R.id.car_routing_options_container);
    mTrailOptionsContainer = mFrame.findViewById(R.id.trail_options_container);

    // Set up routing options toggle
    mRoutingOptionsGearBtn.setOnClickListener(v -> toggleRoutingOptions());
    mAnimToggle = MwmApplication.from(activity.getApplicationContext())
                                .getResources().getInteger(R.integer.anim_default);

    final View menuFrame = activity.findViewById(R.id.menu_frame);
    final PaddingInsetsListener insetsListener = new PaddingInsetsListener.Builder()
        .setInsetsTypeMask(WindowInsetsCompat.Type.systemBars() | WindowInsetsCompat.Type.displayCutout())
        .setExcludeTop()
        .build();
    ViewCompat.setOnApplyWindowInsetsListener(menuFrame, insetsListener);
  }

  @NonNull
  protected View getFrame()
  {
    return mFrame;
  }

  private void toggleRoutingOptions()
  {
    boolean isVisible = UiUtils.isVisible(mCarRoutingOptionsContainer) ||
                       UiUtils.isVisible(mTrailOptionsContainer);

    if (isVisible)
    {
      // Hide options
      hideRoutingOptions();
    }
    else
    {
      // Show options based on current routing mode
      showRoutingOptionsForCurrentMode();
    }
  }

  private void showRoutingOptionsForCurrentMode()
  {
    Router currentRouter = RoutingController.get().getLastRouterType();

    // Hide all options first
    mCarRoutingOptionsContainer.setVisibility(View.GONE);
    mTrailOptionsContainer.setVisibility(View.GONE);

    if (currentRouter == Router.Vehicle)
      showCarRoutingOptions();
    else if (currentRouter == Router.Pedestrian || currentRouter == Router.Bicycle)
      showTrailOptions();

    // Show the container with header
    UiUtils.show(mRoutingOptionsContainer);
    UiUtils.show(mRoutingOptionsHeader);

    ImageView toggleIcon = mFrame.findViewById(R.id.routing_options_toggle);
    toggleIcon.animate().rotation(180).setDuration(mAnimToggle).start();
  }

  private void hideRoutingOptions()
  {
    UiUtils.hide(mCarRoutingOptionsContainer);
    UiUtils.hide(mTrailOptionsContainer);
    UiUtils.hide(mRoutingOptionsContainer);

    // Reset toggle icon
    ImageView toggleIcon = mFrame.findViewById(R.id.routing_options_toggle);
    if (toggleIcon != null)
    {
      toggleIcon.animate().rotation(0).setDuration(mAnimToggle).start();
    }
  }

  private void showCarRoutingOptions()
  {
    UiUtils.show(mCarRoutingOptionsContainer);

    // Initialize car routing options (tolls, unpaved, ferry, motorways)
    initCarRoutingOptions();
  }

  private void showTrailOptions()
  {
    UiUtils.show(mTrailOptionsContainer);

    // Initialize trail options
    initTrailOptions();
  }

  private void initCarRoutingOptions()
  {
    // Initialize car routing switches based on current settings
    SwitchCompat avoidTollsBtn = mFrame.findViewById(R.id.avoid_tolls_btn);
    SwitchCompat avoidUnpavedBtn = mFrame.findViewById(R.id.avoid_unpaved_btn);
    SwitchCompat avoidFerryBtn = mFrame.findViewById(R.id.avoid_ferry_btn);
    SwitchCompat avoidMotorwaysBtn = mFrame.findViewById(R.id.avoid_motorways_btn);

    // Load current settings from RoutingOptions
    if (avoidTollsBtn != null)
      avoidTollsBtn.setChecked(RoutingOptions.hasOption(RoadType.Toll));
    if (avoidUnpavedBtn != null)
      avoidUnpavedBtn.setChecked(RoutingOptions.hasOption(RoadType.Dirty));
    if (avoidFerryBtn != null)
      avoidFerryBtn.setChecked(RoutingOptions.hasOption(RoadType.Ferry));
    if (avoidMotorwaysBtn != null)
      avoidMotorwaysBtn.setChecked(RoutingOptions.hasOption(RoadType.Motorway));

    // Set up listeners with route recalculation
    if (avoidTollsBtn != null)
      avoidTollsBtn.setOnCheckedChangeListener((buttonView, isChecked) -> {
        if (isChecked) RoutingOptions.addOption(RoadType.Toll);
        else RoutingOptions.removeOption(RoadType.Toll);
        triggerRouteRecalculation();
      });

    if (avoidUnpavedBtn != null)
      avoidUnpavedBtn.setOnCheckedChangeListener((buttonView, isChecked) -> {
        if (isChecked) RoutingOptions.addOption(RoadType.Dirty);
        else RoutingOptions.removeOption(RoadType.Dirty);
        triggerRouteRecalculation();
      });

    if (avoidFerryBtn != null)
      avoidFerryBtn.setOnCheckedChangeListener((buttonView, isChecked) -> {
        if (isChecked) RoutingOptions.addOption(RoadType.Ferry);
        else RoutingOptions.removeOption(RoadType.Ferry);
        triggerRouteRecalculation();
      });

    if (avoidMotorwaysBtn != null)
      avoidMotorwaysBtn.setOnCheckedChangeListener((buttonView, isChecked) -> {
        if (isChecked) RoutingOptions.addOption(RoadType.Motorway);
        else RoutingOptions.removeOption(RoadType.Motorway);
        triggerRouteRecalculation();
      });
  }

  private void initTrailOptions()
  {
    // Initialize trail options switches and slider based on current settings
    SwitchCompat preferTrailsBtn = mFrame.findViewById(R.id.prefer_trails_btn);
    View trailPreferenceContainer = mFrame.findViewById(R.id.trail_preference_container);
    SeekBar trailPreferenceSlider = mFrame.findViewById(R.id.trail_preference_slider);

    // Load current settings from TrailRoutingSettings
    TrailRoutingSettings trailSettings = TrailRoutingSettings.LoadFromSettings();

    if (preferTrailsBtn != null)
    {
      preferTrailsBtn.setChecked(trailSettings.m_preferTrails);
      preferTrailsBtn.setOnCheckedChangeListener((buttonView, isChecked) -> {
        trailSettings.m_preferTrails = isChecked;
        TrailRoutingSettings.SaveToSettings(trailSettings);
        if (trailPreferenceContainer != null)
          trailPreferenceContainer.setVisibility(isChecked ? View.VISIBLE : View.GONE);
        triggerRouteRecalculation();
      });
    }

    if (trailPreferenceContainer != null)
      trailPreferenceContainer.setVisibility(trailSettings.m_preferTrails ? View.VISIBLE : View.GONE);

    if (trailPreferenceSlider != null)
    {
      trailPreferenceSlider.setProgress((int) trailSettings.m_trailPreference);
      trailPreferenceSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
          if (fromUser) {
            trailSettings.m_trailPreference = progress;
            TrailRoutingSettings.SaveToSettings(trailSettings);
          }
        }
        @Override public void onStartTrackingTouch(SeekBar seekBar) {}
        @Override public void onStopTrackingTouch(SeekBar seekBar) {
          triggerRouteRecalculation();
        }
      });
    }
  }

  private void setupRouterButtons()
  {
    setupRouterButton(R.id.vehicle, R.drawable.ic_car, this::onVehicleModeSelected);
    setupRouterButton(R.id.pedestrian, R.drawable.ic_pedestrian, this::onPedestrianModeSelected);
//    setupRouterButton(R.id.taxi, R.drawable.ic_taxi, this::onTaxiModeSelected);
    setupRouterButton(R.id.transit, R.drawable.ic_transit, this::onTransitModeSelected);
    setupRouterButton(R.id.bicycle, R.drawable.ic_bike, this::onBicycleModeSelected);
    setupRouterButton(R.id.ruler, R.drawable.ic_ruler_route, this::onRulerModeSelected);
  }

  private void onTransitModeSelected(@NonNull View v)
  {
    RoutingController.get().setRouterType(Router.Transit);
  }

  private void onBicycleModeSelected(@NonNull View v)
  {
    RoutingController.get().setRouterType(Router.Bicycle);
  }

  private void onRulerModeSelected(@NonNull View v)
  {
    RoutingController.get().setRouterType(Router.Ruler);
  }

  private void onPedestrianModeSelected(@NonNull View v)
  {
    RoutingController.get().setRouterType(Router.Pedestrian);
  }

  private void onVehicleModeSelected(@NonNull View v)
  {
    RoutingController.get().setRouterType(Router.Vehicle);
  }

  @Override
  public void onUpClick()
  {
    // Ignore the event if the back and start buttons are pressed at the same time.
    // See {@link #RoutingBottomMenuController.setStartButton()}.
    if (RoutingController.get().isNavigating())
      return;
    RoutingController.get().cancel();
  }

  boolean checkFrameHeight()
  {
    if (mFrameHeight > 0)
      return true;

    mFrameHeight = mFrame.getHeight();
    return (mFrameHeight > 0);
  }

  private void updateProgressLabels()
  {
    RoutingController.BuildState buildState = RoutingController.get().getBuildState();

    final boolean ready = (buildState == RoutingController.BuildState.BUILT);

    if (!ready) 
    {
      mRoutingBottomMenuController.hideAltitudeChartAndRoutingDetails();
      return;
    }

    if (isTransitType())
    {
      TransitRouteInfo info = RoutingController.get().getCachedTransitInfo();
      if (info != null)
        mRoutingBottomMenuController.showTransitInfo(info);
      return;
    }

    if (isRulerType())
    {
      RoutingInfo routingInfo = RoutingController.get().getCachedRoutingInfo();
      if (routingInfo != null)
        mRoutingBottomMenuController.showRulerInfo(Framework.nativeGetRoutePoints(), routingInfo.distToTarget);
      return;
    }

    final boolean showStartButton = !RoutingController.get().isRulerRouterType();
    mRoutingBottomMenuController.setStartButton(showStartButton);
    mRoutingBottomMenuController.showAltitudeChartAndRoutingDetails();
  }

  public void updateBuildProgress(int progress, @NonNull Router router)
  {
    UiUtils.invisible(mProgressVehicle, mProgressPedestrian, mProgressTransit,
                      mProgressBicycle, mProgressRuler);
    WheelProgressView progressView;
    switch (router)
    {
    case Vehicle:
      mRouterTypes.check(R.id.vehicle);
      progressView = mProgressVehicle;
      break;
    case Pedestrian:
      mRouterTypes.check(R.id.pedestrian);
      progressView = mProgressPedestrian;
      break;
    //case Taxi:
    //    {
    //      mRouterTypes.check(R.id.taxi);
    //      progressView = mProgressTaxi;
    //    }
    case Transit:
      mRouterTypes.check(R.id.transit);
      progressView = mProgressTransit;
      break;
    case Bicycle:
      mRouterTypes.check(R.id.bicycle);
      progressView = mProgressBicycle;
      break;
    case Ruler:
      mRouterTypes.check(R.id.ruler);
      progressView = mProgressRuler;
      break;
    default:
        throw new IllegalArgumentException("unknown router: " + router);
    }

    RoutingToolbarButton button = mRouterTypes
        .findViewById(mRouterTypes.getCheckedRadioButtonId());
    button.progress();

    updateProgressLabels();

    if (!RoutingController.get().isBuilding())
    {
      button.complete();
      return;
    }

    UiUtils.show(progressView);
    progressView.setPending(progress == 0);
    if (progress != 0)
      progressView.setProgress(progress);
  }

  private boolean isTransitType()
  {
    return RoutingController.get().isTransitType();
  }

  private boolean isRulerType()
  {
    return RoutingController.get().isRulerRouterType();
  }

  void saveRoutingPanelState(@NonNull Bundle outState)
  {
    mRoutingBottomMenuController.saveRoutingPanelState(outState);
    outState.putBoolean(BUNDLE_HAS_DRIVING_OPTIONS_VIEW, UiUtils.isVisible(mRoutingOptionsContainer));
  }

  void restoreRoutingPanelState(@NonNull Bundle state)
  {
    mRoutingBottomMenuController.restoreRoutingPanelState(state);
    boolean hasView = state.getBoolean(BUNDLE_HAS_DRIVING_OPTIONS_VIEW);
    if (hasView)
      showDrivingOptionView();
  }

  public void showAddStartFrame()
  {
    mRoutingBottomMenuController.showAddStartFrame();
  }

  public void showAddFinishFrame()
  {
    mRoutingBottomMenuController.showAddFinishFrame();
  }

  public void showDrivingOptionView()
  {
    // Show the gear button if we have routing options available
    boolean routingOptionsAvailable = RoutingOptions.hasAnyOptions();
    boolean trailOptionsAvailable = hasTrailOptionsAvailable();
    boolean isRuler = isRulerType();
    boolean hasAnyOptions = (routingOptionsAvailable || trailOptionsAvailable) && !isRuler;

    // Note: For now, always show gear button for testing
    // TODO: Use proper condition logic once everything is working
    if (hasAnyOptions || shouldShowGearButton())
    {
      UiUtils.show(mRoutingOptionsGearBtn);
      // Hide the options container initially - user will tap gear to show it
      UiUtils.hide(mRoutingOptionsContainer);
    }
    else
    {
      UiUtils.hide(mRoutingOptionsGearBtn);
    }
  }

  // Override in RoutingPlanInplaceController to show gear button when routing plan is visible
  protected void showRoutingOptionsGearBtn()
  {
    // Always show gear button when routing plan is visible
    // This ensures users can configure options even before selecting a destination
    UiUtils.show(mRoutingOptionsGearBtn);
    UiUtils.hide(mRoutingOptionsContainer);
  }

  public void hideDrivingOptionsView()
  {
    UiUtils.hide(mRoutingOptionsContainer);
    UiUtils.hide(mRoutingOptionsGearBtn);
    mRoutingPlanListener.onRoutingPlanStartAnimate(UiUtils.isVisible(getFrame()));
  }

  private boolean hasTrailOptionsAvailable()
  {
    Router currentRouter = RoutingController.get().getLastRouterType();
    return currentRouter == Router.Pedestrian || currentRouter == Router.Bicycle;
  }

  // For testing: force show gear button for debugging
  private boolean shouldShowGearButton()
  {
    // For now, always show the gear button for testing
    // TODO: Remove this and use proper logic once everything works
    return true;
  }

  // Trigger route recalculation when routing options change
  private void triggerRouteRecalculation()
  {
    // Only rebuild if there's already a route (has start and end points)
    if (RoutingController.get().getStartPoint() != null && RoutingController.get().getEndPoint() != null)
    {
      RoutingController.get().rebuildLastRoute();
    }
    // If no route exists yet, the new settings will be used when the route is calculated
  }

  public int calcHeight()
  {
    int frameHeight = getFrame().getHeight();
    if (frameHeight == 0)
      return 0;

    View routingOptionsView = mRoutingOptionsContainer;
    int extraOppositeOffset = UiUtils.isVisible(routingOptionsView)
                              ? 0
                              : routingOptionsView.getHeight();

    return frameHeight - extraOppositeOffset;
  }

  private class SelfTerminatedDrivingOptionsLayoutListener implements View.OnLayoutChangeListener
  {
    @Override
    public void onLayoutChange(View v, int left, int top, int right, int bottom, int oldLeft,
                               int oldTop, int oldRight, int oldBottom)
    {
      mRoutingPlanListener.onRoutingPlanStartAnimate(UiUtils.isVisible(getFrame()));
      mRoutingOptionsContainer.removeOnLayoutChangeListener(this);
    }
  }
}

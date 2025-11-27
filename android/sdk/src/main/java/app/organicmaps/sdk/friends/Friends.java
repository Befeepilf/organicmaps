package app.organicmaps.sdk.friends;

import androidx.annotation.Keep;
import androidx.annotation.Nullable;

public final class Friends
{
  private Friends() {}

  @Keep
  public static final class Friend
  {
    public final String userId;
    public final String username;
    public Friend(String userId, String username)
    {
      this.userId = userId;
      this.username = username;
    }
  }

  @Keep
  public static final class FriendsPayload
  {
    public final Friend[] accepted;
    public final Friend[] incoming;
    public final Friend[] outgoing;
    public FriendsPayload(Friend[] accepted, Friend[] incoming, Friend[] outgoing)
    {
      this.accepted = accepted;
      this.incoming = incoming;
      this.outgoing = outgoing;
    }
  }

  public static native FriendsPayload nativeGetLists();
  public static native void nativeRefresh();
  public static native Friend[] nativeSearchByUsername(String query);
  public static native boolean nativeSendRequest(String userId);
  public static native boolean nativeAcceptRequest(String userId);
  public static native boolean nativeCancelRequest(String userId);
}
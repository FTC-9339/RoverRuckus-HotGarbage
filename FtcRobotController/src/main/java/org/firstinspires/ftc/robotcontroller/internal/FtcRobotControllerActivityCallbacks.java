package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Bundle;

import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.robotcore.eventloop.EventLoop;

import java.util.ArrayList;

public final class FtcRobotControllerActivityCallbacks {
    private static final FtcRobotControllerActivityCallbacks ourInstance = new FtcRobotControllerActivityCallbacks();

    public static FtcRobotControllerActivityCallbacks getInstance() {
        return ourInstance;
    }

    private FtcRobotControllerActivityCallbacks() {}

    private final ArrayList<OnCreateCallback> onCreateCallbacks = new ArrayList<>();
    private final ArrayList<OnServiceBindCallback> onServiceBindCallbacks = new ArrayList<>();
    private final ArrayList<OnDestroyCallback> onDestroyCallbacks = new ArrayList<>();
    private final ArrayList<AttachEventLoopCallback> attachEventLoopCallbacks = new ArrayList<>();

    /**
     * Add your own callback to be called on creation of the main RC app activity.
     *
     * <p>
     *     This is useful for starting up pervasive services that last during the duration of the app,
     *     such as web servers or other unique services.
     * </p>
     *
     * @param callback An object implementing the {@link OnCreateCallback} interface.
     */
    public void addCreateCallback(OnCreateCallback callback) {
        onCreateCallbacks.add(callback);
    }

    /**
     * Add your own callback to be called in order to bind services.
     *
     * <p>
     *     This is useful for binding pervasive services that last during the duration of the app,
     *     especially web servers.
     * </p>
     *
     * @param callback An object implementing the {@link OnServiceBindCallback} interface.
     */
    public void addServiceBindCallback(OnServiceBindCallback callback) {
        onServiceBindCallbacks.add(callback);
    }

    /**
     * Add your own callback to be called on the destruction of the main RC app activity.
     *
     * <p>
     *     This is useful for shutting down pervasive services that have lasted during the duration of the app,
     *     such as web servers or other unique services.
     * </p>
     *
     * @param callback An object implementing the {@link OnDestroyCallback} interface.
     */
    public void addDestroyCallback(OnDestroyCallback callback) {
        onDestroyCallbacks.add(callback);
    }

    /**
     * Add your own callback to be called in order to attach an eventloop to a service.
     *
     * <p>
     *     This is useful for creating an "interop" between a custom service and the robot controller,
     *     allowing for things like web servers.
     * </p>
     *
     * @param callback An object implementing the {@link AttachEventLoopCallback} interface.
     */
    public void addAttachEventLoopCallback(AttachEventLoopCallback callback) { attachEventLoopCallbacks.add(callback); }

    public ArrayList<OnCreateCallback> getOnCreateCallbacks() {
        return onCreateCallbacks;
    }

    public ArrayList<OnServiceBindCallback> getOnServiceBindCallbacks() { return onServiceBindCallbacks; }

    public ArrayList<OnDestroyCallback> getOnDestroyCallbacks() {
        return onDestroyCallbacks;
    }

    public ArrayList<AttachEventLoopCallback> getAttachEventLoopCallbacks() { return attachEventLoopCallbacks; }

    /**
     * A simple interface containing a callback to be called on creation.
     */
    public interface OnCreateCallback {
        /**
         * Code to be ran once on creation.
         *
         * TODO: Possibly expose the inner fields of the activity through an exposer class and pass that through for more functionality?
         *
         * @param savedInstanceState A passthrough of the {@link Bundle} provided by the {@link FtcRobotControllerActivity} onCreate() method.
         */
        void onCreate(Bundle savedInstanceState);
    }

    /**
     * A simple interface containing a callback to be called to bind services.
     */
    public interface OnServiceBindCallback {
        /**
         * Code to be ran to bind services.
         *
         * @param service A passthrough of the {@link FtcRobotControllerService} provided by the {@link FtcRobotControllerActivity} onServiceBind() method.
         */
        void onServiceBind(final FtcRobotControllerService service);
    }

    /**
     * A simple interface containing a callback to attach an eventloop.
     */
    public interface AttachEventLoopCallback {
        /**
         * Code to be ran in order to attach an eventloop.
         *
         * @param eventLoop An {@link EventLoop} passed through by the {@link FtcRobotControllerActivity}.
         */
        void attachEventLoop(EventLoop eventLoop);
    }

    /**
     * A simple interface containing a callback to be called on destruction of the {@link FtcRobotControllerActivity}.
     */
    public interface OnDestroyCallback {
        /**
         * Code to be ran upon destruction.
         */
        void onDestroy();
    }
}

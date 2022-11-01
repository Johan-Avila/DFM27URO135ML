#include "utility.h"

class MonoDFM
{
public:
    ParamServer param_server;

    ros::Publisher pub_img_cam;

    GstElement *pipeline;
    GstElement *cam;
    GstElement *capsfilter;
    GstElement *appsink;

    MonoDFM()
    {
        string pip_des;
        pip_des.append("tcambin name=camera serial=");
        pip_des.append(this->param_server.serial_cam);
        pip_des.append(" use-dutils=true ");
        pip_des.append("! capsfilter name=filter ");
        pip_des.append("! queue max-size-buffers=2 leaky=downstream ");
        pip_des.append("! appsink name=sink");

        const char *pipeline_description = pip_des.c_str();

        GError *erro = NULL;
        this->pipeline = gst_parse_launch(pipeline_description, &erro);
        this->cam = gst_bin_get_by_name(GST_BIN(this->pipeline), "camera");
        this->capsfilter = gst_bin_get_by_name(GST_BIN(this->pipeline), "filter");
        this->appsink = gst_bin_get_by_name(GST_BIN(this->pipeline), "sink");

        if (this->pipeline == NULL)
        {
            ROS_INFO_STREAM("Could not create pipeline. Cause:" << erro->message);
            exit(1);
        }
        if (this->cam == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve cam from pipeline.");
            exit(1);
        }
        if (this->capsfilter == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve capsfilter from pipeline.");
            exit(1);
        }
        if (this->appsink == NULL)
        {
            ROS_INFO_STREAM("Could not retrieve appsink from pipeline.");
            exit(1);
        }
    }

    void setting_publisher()
    {
        ros::NodeHandle nh;
        this->pub_img_cam = nh.advertise<sensor_msgs::Image>(
            "dfm27uro135ml/cam/image_raw", 1);
    }

    void setting_camera_before_PLAYING()
    {
        this->param_server.set_property(this->cam, "Brightness", "Brightness", "int");
        this->param_server.set_property(this->cam, "GPIn", "GPIn", "int");
        this->param_server.set_property(this->cam, "GPOut", "GPOut", "int");
        this->param_server.set_property(this->cam, "GainDB100", "Gain (dB/100)", "int");
        this->param_server.set_property(this->cam, "OffsetAutoCenter", "Offset Auto Center", "boolean");
        this->param_server.set_property(this->cam, "OffsetX", "Offset X", "int");
        this->param_server.set_property(this->cam, "OffsetY", "Offset Y", "int");
        this->param_server.set_property(this->cam, "OverrideScanningMode", "Override Scanning Mode", "int");
        this->param_server.set_property(this->cam, "StrobeEnable", "Strobe Enable", "boolean");
        this->param_server.set_property(this->cam, "StrobeExposure", "Strobe Exposure", "boolean");
        this->param_server.set_property(this->cam, "StrobePolarity", "Strobe Polarity", "boolean");
        this->param_server.set_property(this->cam, "TriggerDelayUs", "Trigger Delay (us)", "int");
        this->param_server.set_property(this->cam, "TriggerGlobalResetRelease", "Trigger Global Reset Release", "boolean");
        this->param_server.set_property(this->cam, "TriggerMode", "Trigger Mode", "boolean");
        this->param_server.set_property(this->cam, "cameraWhitebalance", "camera-whitebalance", "boolean");
        this->param_server.set_property(this->cam, "whitebalanceAuto", "whitebalance-auto", "boolean");
        this->param_server.set_property(this->cam, "whitebalanceBlue", "whitebalance-blue", "int");
        this->param_server.set_property(this->cam, "whitebalanceGreen", "whitebalance-green", "int");
        this->param_server.set_property(this->cam, "whitebalanceModuleEnabled", "whitebalance-module-enabled", "boolean");
        this->param_server.set_property(this->cam, "whitebalanceRed", "whitebalance-red", "int");
    }

    void setting_camera_during_PLAYING()
    {
        this->param_server.set_property(this->cam, "BrightnessReference", "Brightness Reference", "int");
        this->param_server.set_property(this->cam, "ExposureAuto", "Exposure Auto", "boolean");
        this->param_server.set_property(this->cam, "ExposureMin", "Exposure Min", "int");
        this->param_server.set_property(this->cam, "ExposureMax", "Exposure Max", "int");
        this->param_server.set_property(this->cam, "ExposureTimeUs", "Exposure Time (us)", "int");
        this->param_server.set_property(this->cam, "GainAuto", "Gain Auto", "boolean");
        this->param_server.set_property(this->cam, "GainMin", "Gain Min", "double");
        this->param_server.set_property(this->cam, "GainMax", "Gain Max", "double");
        this->param_server.set_property(this->cam, "Gain", "Gain", "int");
        this->param_server.set_property(this->cam, "ExposureROILeft", "Exposure ROI Left", "int");
        this->param_server.set_property(this->cam, "ExposureROIWidth", "Exposure ROI Width", "int");
        this->param_server.set_property(this->cam, "ExposureROITop", "Exposure ROI Top", "int");
        this->param_server.set_property(this->cam, "ExposureROIHeight", "Exposure ROI Height", "int");

        gst_object_unref(this->cam);
    }

    void setting_capsfilter()
    {
        this->param_server.set_format(this->capsfilter);
        gst_object_unref(this->capsfilter);
    }

    void setting_appsink()
    {
        g_object_set(G_OBJECT(this->appsink), "sync", FALSE, NULL);
        g_object_set(G_OBJECT(this->appsink), "emit-signals", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink), "drop", TRUE, NULL);
        g_object_set(G_OBJECT(this->appsink), "max-buffers", 4, NULL);
    }
};

gboolean block_until_playing(GstElement *pipeline)
{
    while (TRUE)
    {
        GstState state;
        GstState pending;

        // wait 0.1 seconds for something to happen
        GstStateChangeReturn ret = gst_element_get_state(pipeline, &state, &pending, 100000000);

        if (ret == GST_STATE_CHANGE_SUCCESS)
        {
            return TRUE;
        }
        else if (ret == GST_STATE_CHANGE_FAILURE)
        {
            printf("Failed to change state %s %s %s\n",
                   gst_element_state_change_return_get_name(ret),
                   gst_element_state_get_name(state),
                   gst_element_state_get_name(pending));

            return FALSE;
        }
    }
}

static GstFlowReturn publish_img(GstElement *appsink, MonoDFM *mono_DFM)
{
    GstSample *sample = NULL;
    g_signal_emit_by_name(appsink, "pull-sample", &sample, NULL);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo info;
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    sensor_msgs::Image img;
    img.header.frame_id = "cam";
    img.height = mono_DFM->param_server.height;
    img.width = mono_DFM->param_server.width;
    img.encoding = mono_DFM->param_server.ros_encoding_type;
    img.data.assign(info.data, info.data + info.size);
    img.step = (mono_DFM->param_server.width) * (mono_DFM->param_server.ros_num_channels);
    img.header.stamp = ros::Time::now();
    mono_DFM->pub_img_cam.publish(img);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dfm_27uro135_ml");

    gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(&argc, &argv);

    MonoDFM mono_DFM;
    mono_DFM.setting_publisher();
    mono_DFM.setting_camera_before_PLAYING();
    mono_DFM.setting_capsfilter();
    mono_DFM.setting_appsink();

    g_signal_connect(mono_DFM.appsink, "new-sample", G_CALLBACK(publish_img), &mono_DFM);

    gst_object_unref(mono_DFM.appsink);

    gst_element_set_state(mono_DFM.pipeline, GST_STATE_PLAYING);

    if (!block_until_playing(mono_DFM.pipeline))
    {
        ROS_INFO_STREAM("Unable to start pipeline.");
    }

    mono_DFM.setting_camera_during_PLAYING();

    ROS_INFO_STREAM("\033[1;32m-> PublisherImg.\033[0m");
    ROS_INFO_STREAM("Press Ctrl-C to stop.");
    // while (ros::ok())
    // {
    // }
    getchar();

    gst_element_set_state(mono_DFM.pipeline, GST_STATE_NULL);

    gst_object_unref(mono_DFM.pipeline);
    return 0;
}

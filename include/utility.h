#pragma once

#ifndef ULTILITY_H
#define ULTILITY_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/videooverlay.h>
#include <stdio.h>
#include <unistd.h>
#include "tcamprop.h"

#include <string>

using namespace std;

class ParamServer
{

public:
    ros::NodeHandle nh;

    string pwd_base_serial = "/dfm27uro135ml/serial/";
    string pwd_base_formats = "/dfm27uro135ml/formats/";
    string pwd_base_properties = "/dfm27uro135ml/properties/";
    string pwd_base_ros_encoding_type = "/dfm27uro135ml/rosEncodingType/";
    string pwd_base_ros_num_channels = "/dfm27uro135ml/rosNumChannels/";

    string serial_cam;

    string ros_encoding_type;
    int32_t width;
    int32_t height;
    int ros_num_channels;

    ParamServer()
    {
        string pwd_serial_cam;
        pwd_serial_cam.append(pwd_base_serial);
        pwd_serial_cam.append("cam");

        nh.param<std::string>(pwd_serial_cam, serial_cam, "");

        string pwd_width;
        pwd_width.append(pwd_base_formats);
        pwd_width.append("width");

        string pwd_height;
        pwd_height.append(pwd_base_formats);
        pwd_height.append("height");

        nh.param<int32_t>(pwd_width, width, 640);
        nh.param<int32_t>(pwd_height, height, 480);

        string pwd_format;
        pwd_format.append(pwd_base_formats);
        pwd_format.append("format");

        string format;
        nh.param<std::string>(pwd_format, format, "GRAY8");

        string pwd_ros_encoding_type;
        pwd_ros_encoding_type.append(pwd_base_ros_encoding_type);
        pwd_ros_encoding_type.append(format);

        nh.param<std::string>(pwd_ros_encoding_type, ros_encoding_type, "mono8");

        string pwd_ros_num_channels;
        pwd_ros_num_channels.append(pwd_base_ros_num_channels);
        pwd_ros_num_channels.append(format);

        nh.param<int>(pwd_ros_num_channels, ros_num_channels, 3);
    }

    void set_property(GstElement *cam, const char *name_ros, const char *name, const char *type)
    {
        GValue property = G_VALUE_INIT;
        string pwd_property;
        pwd_property.append(pwd_base_properties);
        pwd_property.append(name_ros);

        if (strcmp(type, "int") == 0)
        {
            g_value_init(&property, G_TYPE_INT);
            int property_tmp;
            nh.param<int>(pwd_property, property_tmp, 1);
            g_value_set_int(&property, property_tmp);
            tcam_prop_set_tcam_property(TCAM_PROP(cam), name, &property);
        }
        else if (strcmp(type, "boolean") == 0)
        {
            g_value_init(&property, G_TYPE_BOOLEAN);
            bool property_tmp;
            nh.param<bool>(pwd_property, property_tmp, false);
            g_value_set_boolean(&property, property_tmp);
            tcam_prop_set_tcam_property(TCAM_PROP(cam), name, &property);
        }
        else if (strcmp(type, "double") == 0)
        {
            g_value_init(&property, G_TYPE_DOUBLE);
            double property_tmp;
            nh.param<double>(pwd_property, property_tmp, 1.0);
            g_value_set_double(&property, property_tmp);
            tcam_prop_set_tcam_property(TCAM_PROP(cam), name, &property);
        }
        else
        {
            printf("type no valido: %s", type);
        }
        g_value_unset(&property);
    }

    void set_format(GstElement *capsfilter)
    {
        string pwd_name;
        pwd_name.append(pwd_base_formats);
        pwd_name.append("mame");

        string pwd_format;
        pwd_format.append(pwd_base_formats);
        pwd_format.append("format");

        string pwd_width;
        pwd_width.append(pwd_base_formats);
        pwd_width.append("width");

        string pwd_height;
        pwd_height.append(pwd_base_formats);
        pwd_height.append("height");

        string pwd_fps;
        pwd_fps.append(pwd_base_formats);
        pwd_fps.append("fps");

        string name;
        string format;
        int32_t width;
        int32_t height;
        int32_t fps;

        nh.param<std::string>(pwd_name, name, "video/x-raw");
        nh.param<std::string>(pwd_format, format, "GRAY8");
        nh.param<int>(pwd_width, width, 640);
        nh.param<int>(pwd_height, height, 480);
        nh.param<int>(pwd_fps, fps, 10);

        const char *name_char = name.c_str();
        const char *format_char = format.c_str();

        GstStructure *structure = gst_structure_from_string(name_char, NULL);
        gst_structure_set(structure,
                          "format", G_TYPE_STRING, format_char,
                          "width", G_TYPE_INT, width,
                          "height", G_TYPE_INT, height,
                          "framerate", GST_TYPE_FRACTION, fps, 1,
                          NULL);

        GstCaps *caps = gst_caps_new_empty();

        gst_caps_append_structure(caps, structure);
        g_object_set(G_OBJECT(capsfilter), "caps", caps, NULL);
        gst_caps_unref(caps);
    }
};
#endif
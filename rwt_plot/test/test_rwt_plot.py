#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Kei Okada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Copyright holder. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import argparse
import sys
import time
import rospy
import rostest
import unittest

from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.support.ui import Select

import pkg_resources
selenium_version = pkg_resources.get_distribution("selenium").version
# Check if selenium version is greater than 4.3.0
if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
    from selenium.webdriver.support.ui import WebDriverWait
    from selenium.webdriver.common.by import By

from std_msgs.msg import Float64

CLASSNAME = 'rwt_plot'

class TestRwtPlot(unittest.TestCase):

    def sin_cb(self, msg):
        self.sin_msg = msg
        self.sin_msg_received = self.sin_msg_received + 1

    def __init__(self, *args):
        super(TestRwtPlot, self).__init__(*args)
        rospy.init_node('test_rwt_plot')

    def setUp(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--no-headless', action='store_true',
                            help='start webdriver with headless mode')
        args, unknown = parser.parse_known_args()

        self.sin_msg = None
        self.sin_msg_received = 0

        rospy.Subscriber('/sin', Float64, self.sin_cb)
        self.url_base = rospy.get_param("url_roswww_testserver")

        opts = webdriver.firefox.options.Options()
        if not args.no_headless:
            opts.add_argument('-headless')
        self.browser = webdriver.Firefox(options=opts)

        self.wait = webdriver.support.ui.WebDriverWait(self.browser, 10)
        # maximize screen
        if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
            self.browser.fullscreen_window()
        else:
            self.browser.find_element_by_tag_name("html").send_keys(Keys.F11)

    def tearDown(self):
        try:
            self.browser.close()
            self.browser.quit()
        except:
            pass

    def test_rwt_plot(self):
        url = '%s/rwt_plot' % (self.url_base)
        rospy.logwarn("Accessing to %s" % url)

        self.browser.get(url)

        # check settings
        self.wait.until(EC.presence_of_element_located((By.ID, "button-ros-master-settings")))
        settings = self.find_element_by_id("button-ros-master-settings")
        self.assertIsNotNone(settings, "Object id=button-ros-master-settings not found")
        settings.click()

        self.wait.until(EC.presence_of_element_located((By.ID, "input-ros-master-uri")))
        uri = self.find_element_by_id("input-ros-master-uri")
        self.assertIsNotNone(uri, "Object id=input-ros-master-uri not found")
        uri.clear();
        uri.send_keys('ws://localhost:9090/')

        self.wait.until(EC.presence_of_element_located((By.ID, "button-ros-master-connect")))
        connect = self.find_element_by_id("button-ros-master-connect")
        self.assertIsNotNone(connect, "Object id=button-ros-master-connect")
        connect.click()

        # wait for /sin topic
        topic_text = ''
        while topic_text == '':
            time.sleep(1)
            self.wait.until(EC.presence_of_element_located((By.ID, "topic-select")))
            topic = self.find_element_by_id("topic-select")
            self.assertIsNotNone(topic, "Object id=topic-select not found")
            topic_text = topic.text
        self.assertTrue(u'/sin' in topic_text)

        Select(topic).select_by_value('/sin')

        # select field
        self.wait.until(EC.presence_of_element_located((By.ID, "field-accessor")))
        field = self.find_element_by_id("field-accessor")
        self.assertIsNotNone(field, "Object id=field-accessor not found")
        field.clear()
        field.send_keys('data')

        self.wait.until(EC.presence_of_element_located((By.ID, "topic-field-button")))
        plot = self.find_element_by_id("topic-field-button")
        self.assertIsNotNone(plot, "Object id=topic-field-button")
        plot.click()

        # check topic type
        self.wait.until(EC.presence_of_element_located((By.ID, "message-detail")))
        topic_type = self.find_element_by_id("message-detail")
        self.assertIsNotNone(topic_type, "Object id=message-detail")
        # self.assertTrue(u'"data": "float64"' in topic_type.text) ## it sometimes fails

        # check plot is updated
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "g.x")))
        x_axis = self.find_element_by_css_selector("g.x")
        self.assertIsNotNone(x_axis, "Object id=x_axis")
        x_axis_value = x_axis.text
        loop = 0
        x_axis_value_updated = 0
        while loop < 20:
            loop = loop + 1
            time.sleep(1)
            x_axis = self.find_element_by_css_selector("g.x")
            rospy.logwarn("check if tick updated {} < {} ({})".format(x_axis_value, x_axis.text, x_axis_value_updated))
            if x_axis_value != x_axis.text:
                x_axis_value_updated =  x_axis_value_updated + 1
                if x_axis_value_updated >= 4:
                    break
            x_axis_value = x_axis.text

        self.assertNotEqual(x_axis_value, x_axis.text)
            
    def find_element_by_id(self, name):
        if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
            return self.browser.find_element(By.ID, name)
        else:
            return self.browser.find_element_by_id(name)

    def find_element_by_css_selector(self, name):
        if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
            return self.browser.find_element(By.CSS_SELECTOR, name)
        else:
            return self.browser.find_element_by_css_selector(name)


if __name__ == '__main__':
    try:
        rostest.run('rwt_plot', CLASSNAME, TestRwtPlot, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))

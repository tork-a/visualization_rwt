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
    from selenium.webdriver.common.by import By

from sound_play.msg import SoundRequest

CLASSNAME = 'rwt_app_chooser'

class TestRwtAppChooser(unittest.TestCase):

    def robotsound_cb(self, msg):
        rospy.logwarn("{} received".format(msg))
        self.robotsound_msg = msg
        self.robotsound_msg_received = self.robotsound_msg_received + 1

    def __init__(self, *args):
        super(TestRwtAppChooser, self).__init__(*args)
        rospy.init_node('test_rwt_app_chooser')

    def setUp(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--no-headless', action='store_true',
                            help='start webdriver with headless mode')
        args, unknown = parser.parse_known_args()

        self.robotsound_msg = None
        self.robotsound_msg_received = 0

        rospy.Subscriber('/robotsound', SoundRequest, self.robotsound_cb)
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

    def test_rwt_app_chooser(self):
        url = '%s/rwt_app_chooser' % (self.url_base)
        rospy.logwarn("Accessing to %s" % url)

        self.browser.get(url)

        # Add Robot
        self.wait.until(EC.presence_of_element_located((By.ID, "add")))
        add_robot = self.find_element_by_id("add")
        self.assertIsNotNone(add_robot, "Object id=add not found")
        add_robot.click()

        self.wait.until(EC.presence_of_element_located((By.ID, "robot-name")))
        name = self.find_element_by_id("robot-name")
        self.assertIsNotNone(name, "Object id=robot-name not found")
        name.clear();
        name.send_keys('pr1012')

        self.wait.until(EC.presence_of_element_located((By.ID, "robot-uri")))
        uri = self.find_element_by_id("robot-uri")
        self.assertIsNotNone(uri, "Object id=robot-uri not found")
        uri.clear();
        uri.send_keys('ws://localhost:9090/')

        self.wait.until(EC.presence_of_element_located((By.ID, "add-btn")))
        add = self.find_element_by_id("add-btn")
        self.assertIsNotNone(add, "Object id=add-btn not found")
        add.click()

        # confirm
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "a[class='btn btn-flat primary btn-confirm']")))
        confirm = self.find_element_by_css_selector("a[class='btn btn-flat primary btn-confirm']")
        self.assertIsNotNone(confirm, "Object a[class='btn btn-flat primary btn-confirm']")
        confirm.click()

        # Select Robot
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "span[class='title']")))
        select_robot = self.find_element_by_css_selector("span[class='title']")
        self.assertIsNotNone(select_robot, "Object div[class='item-content']/span[class='title']")
        self.assertTrue(u'pr1012' in select_robot.text)
        rospy.logwarn("Selected {} robot".format(select_robot.text))

        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "li[class='item-expanded robot-list-item']")))
        select = self.find_element_by_css_selector("li[class='item-expanded robot-list-item']")
        self.assertIsNotNone(select, "Object li[class='item-expanded robot-list-item']")
        select.click()

        # Select Task
        self.wait.until(EC.presence_of_element_located((By.XPATH, "//div/span[@class='Title' and contains(text(), 'Hello World')]")))
        task_text = self.find_element_by_xpath("//div/span[@class='Title' and contains(text(), 'Hello World')]")
        self.assertIsNotNone(task_text, "Object span[class='Title']")
        self.assertTrue(u'Hello World' in task_text.text, "Hello World is not found in {}".format(task_text))
        task_text.click()
        rospy.logwarn("Selected {} task".format(task_text.text))

        # input user name
        rospy.logerr('Username Input')
        self.wait.until(EC.presence_of_element_located((By.XPATH, '//h3[text()="Register user name"]')))
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "input[type='text'][placeholder]")))
        user = self.find_element_by_css_selector("input[type='text'][placeholder]")
        self.assertIsNotNone(user, "Object input[type='text'][placeholder]")
        user.send_keys('user')
        rospy.logerr('Done')

        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "a[class='btn btn-flat primary btn-confirm']")))
        confirm = self.find_element_by_css_selector("a[class='btn btn-flat primary btn-confirm']")
        self.assertIsNotNone(confirm, "Object a[class='btn btn-flat primary btn-confirm']")
        confirm.click()
        rospy.logerr('Done')

        # input app arguments
        rospy.logerr('App args Input')
        self.wait.until(EC.presence_of_element_located((By.XPATH, '//h3[text()="App arguments"]')))
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "input[type='text'][placeholder]")))
        args_candidate = self.find_elements_by_css_selector("input[type='text'][placeholder]")
        args = None
        for a in args_candidate:
            if a.is_displayed():
                args = a
        self.assertTrue(args is not None)
        self.assertIsNotNone(user, "Object input[type='text'][placeholder]")
        args.send_keys('{}')
        rospy.logerr('Done')

        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "a[class='btn btn-flat primary btn-confirm']")))
        ok_candidate = self.find_elements_by_css_selector("a[class='btn btn-flat primary btn-confirm']")
        ok = None
        for o in ok_candidate:
            if o.is_displayed():
                ok = o
        self.assertTrue(ok is not None)
        self.assertIsNotNone(ok, "Object a[class='btn btn-flat primary btn-confirm']")
        ok.click()
        rospy.logerr('Done')

        # confirm
        self.wait.until(EC.presence_of_element_located((By.XPATH,"//h3[contains(text(), 'Launch application')]")))
        self.wait.until(EC.presence_of_element_located((By.XPATH,"//p[contains(text(), 'Launch Hello World?')]")))
        self.wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "a[class='btn btn-flat primary btn-confirm']")))
        confirm = self.find_element_by_css_selector("a[class='btn btn-flat primary btn-confirm']")
        self.assertIsNotNone(confirm, "Object a[class='btn btn-flat primary btn-confirm']")
        self.browser.execute_script("arguments[0].scrollIntoView();", confirm)
        self.browser.execute_script("arguments[0].click();", confirm)
        
        # check task works
        loop = 0
        while loop < 30 and self.robotsound_msg == None:
            loop = loop + 1
            time.sleep(1)
            rospy.logwarn("wait for message...")
            try:
                # if we staill have Launch application window, retry execute script
                if self.find_element_by_xpath("//h3[contains(text(), 'Launch application')]"):
                    confirm = self.find_element_by_css_selector("a[class='btn btn-flat primary btn-confirm']")
                    self.browser.execute_script("arguments[0].scrollIntoView();", confirm)
                    self.browser.execute_script("arguments[0].click();", confirm)
            except:
                pass

        self.assertNotEqual(self.robotsound_msg, None, "robotsound_msg did not received")
            
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

    def find_elements_by_css_selector(self, name):
        if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
            return self.browser.find_elements(By.CSS_SELECTOR, name)
        else:
            return self.browser.find_elements_by_css_selector(name)

    def find_element_by_xpath(self, name):
        if pkg_resources.parse_version(selenium_version) >= pkg_resources.parse_version("4.3.0"):
            return self.browser.find_element(By.XPATH, name)
        else:
            return self.browser.find_element_by_xpath(name)


if __name__ == '__main__':
    try:
        rostest.run('rwt_app_chooser', CLASSNAME, TestRwtAppChooser, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))

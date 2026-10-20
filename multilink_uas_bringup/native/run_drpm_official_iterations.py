#!/usr/bin/env python3
"""Predeclared 30-iteration check, separate from the 10-iteration comparison."""
import run_drpm_official as campaign
if __name__=='__main__':
    campaign.OUT=campaign.OUT/'iteration_check_30'
    campaign.rospy.init_node('drpm_iteration_driver',anonymous=True,disable_signals=True)
    for label in campaign.LABELS:campaign.run('official_both',label,iterations=30)

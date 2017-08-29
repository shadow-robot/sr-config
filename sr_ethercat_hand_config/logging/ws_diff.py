#!/usr/bin/env python

import os
import rospy
import datetime

def is_dir(path):
    return os.path.exists(path) and not (os.path.isfile(path) or os.path.islink(path))

def recursive_diff(path):
    if not is_dir(path):
        return ""
    print path
    current_directory = os.getcwd()
    os.chdir(path)
    output = ""
    if is_dir("./.git"):
        print path

    os.chdir(current_directory)
    return output

rospy.init_node("ws_diff")


package_path = os.environ['ROS_PACKAGE_PATH']
package_dirs = filter (lambda x: ("/opt/ros" not in x), package_path.split(":"))

run_time = datetime.datetime.fromtimestamp(rospy.get_rostime().secs)
name_string = "ws_diff_%04d-%02d-%02d-%02d-%02d-%02d" % (
    run_time.year, run_time.month, run_time.day, run_time.hour, run_time.minute, run_time.second)

output_dir = rospy.get_param("~log_directory", ".")

output_path = "%s/wsdiff_%s.gz" % (output_dir, name_string)

output = package_path

for directory in package_dirs:
    output = output + recursive_diff(directory)


# my $z = new IO::Compress::Gzip $out_file;

# $z->print($output);

# #--------------------------------------------------------------------

# sub recursive_diff
# {
#     my ($path) = @_;
#     return if (! -d $path);

#     my $output = "";

#     if (-d "$path/.git")
#     {
#         chdir($path);
#         $output .=  "\n\n-----$path-----\n";
#         $output .=  `git show`;
#         $output .=  `git status`;
#         $output .=  `git diff`;
#     }

#     opendir(PATH, $path);
#     my @sub_dir = grep {/[^\.]/ && $_ ne ".git" && -d "$path/$_"} readdir(PATH);
#     @sub_dir = map {"$path/$_"} @sub_dir;

#     for (@sub_dir)
#     {
#         $output .= recursive_diff($_);
#     }
#     return $output;
# }

#!/usr/bin/perl

use strict;
use IO::Compress::Gzip qw/gzip/;
use POSIX qw/strftime/;

my @dirs = grep {$_ !~ /^\/opt/ } split (":", $ENV{ROS_PACKAGE_PATH});

my $date_time = strftime "%Y-%m-%d-%H-%M-%S", localtime;

my $output = "$ENV{ROS_PACKAGE_PATH}\n";

my $output_dir = ((@ARGV > 1) && -d $ARGV[1]) ? $ARGV[1] : $ENV{PWD};

my $out_file = "$output_dir/wsdiff_$date_time.gz";

for (@dirs)
{
    $output .= recursive_diff($_);
}

my $z = new IO::Compress::Gzip $out_file;

sub recursive_diff
{
    my ($path) = @_;
    return if (! -d $path);

    my $output = "";
    
    if (-d "$path/.git")
    {
        chdir($path);
        $output .=  "\n\n-----$path-----\n";
        $output .=  `git show`;
        $output .=  `git status`;
        $output .=  `git diff`;
    }
    
    opendir(PATH, $path);
    my @sub_dir = grep {/[^\.]/ && $_ ne ".git" && -d "$path/$_"} readdir(PATH);
    @sub_dir = map {"$path/$_"} @sub_dir;

    for (@sub_dir)
    {
        $output .= recursive_diff($_);
    }
    return $output;
}

$z->print($output);

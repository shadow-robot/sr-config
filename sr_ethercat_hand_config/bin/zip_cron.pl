#!/usr/bin/perl

use strict;
my $directory = "/home/hand/log";

chdir($directory);

opendir (my $directory_handle, ".");

my @log_files = sort(grep {/\.log$/ && -f $_} readdir ($directory_handle));

pop (@log_files); # remove the most recent one

if (@log_files)
{
    my $file_name = time().".tgz";
    my $file_list = join(" ", @log_files);
    `tar -zcf $file_name $file_list`;
    unlink($_) for @log_files;
}

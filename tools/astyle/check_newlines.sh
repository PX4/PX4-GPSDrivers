#!/usr/bin/env bash
# Every tracked text file ends in exactly one newline.
status=0

for file in $(git grep --cached -Il ''); do

	if [ -n "$(tail -c 1 "$file")" ]; then
		echo "No newline at end of $file"
		status=1
	elif [ -z "$(tail -c 2 "$file" | head -c 1)" ]; then
		echo "Multiple newlines at end of $file"
		status=1
	fi
done

exit $status

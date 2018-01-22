#!/bin/bash

set -e
# set -x

cd $CI_SOURCE_PATH

PUBLISHED_DIR=${PUBLISHED_DIR:-public}

DEST_DIR=$(readlink -f $CI_SOURCE_PATH/$PUBLISHED_DIR)
if [ -d $DEST_DIR ]; then
  rm -rf $DEST_DIR
fi
mkdir -p $DEST_DIR


catkin_topological_order | while read line; do
  PKGNAME=$(echo $line | awk '{print $1}')
  PKGPATH="`readlink -f $(echo $line | awk '{print $2}')`"
  if [ -d "$PKGPATH/www" ]; then
    cp -r "$PKGPATH/www" $DEST_DIR/$PKGNAME
    echo "Copied $PKGPATH/www -> $DEST_DIR/$PKGNAME"
  else
    echo "Package $PKGNAME does not have www directory"
  fi
done
## add index html for gh-pages
echo "Copied master:READEME.md -> gh-pages:README.md"
sed 's@(\(.*\)/README.md)@(\1)@' README.md > $DEST_DIR/README.md
echo "All copy done!"

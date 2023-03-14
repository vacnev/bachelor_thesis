wrap_line()
{
    indent=$(printf "$1" | wc -c);
    width=$((${COLUMNS:-80} - $indent - 1));
    c1="$1"; shift
    c2="$@"
    nl=
    while test -n "$c2"; do
        printf "$nl%-${indent}s%.${width}s" "$c1" "$c2"
        c2="$(echo "$c2" | cut -c$(($width + 1))-)";
        c1=
        nl="\\\\""\n"; # "
    done
    echo
}

wrap()
{
    while IFS= read -r line; do wrap_line "" "$line"; done
}

log()
{
    test -z "$out" && out=check.$$.out

    wrap_line "$ " "$@" >> $out
    "$@" > $out.tmp 2>&1
    err=$?
    count=$(wc -l < $out.tmp)
    limit=${max_lines:-50}

    if test $count -gt $limit; then
        head -n 10 < $out.tmp | wrap >> $out
        echo "[$(( $count - $limit )) lines of output suppressed]" >> $out
        tail -n $(( $limit - 10 )) < $out.tmp | wrap >> $out
    else
        wrap < $out.tmp >> $out
    fi

    rm -f $out.tmp
    return $err
}

fail()
{
    failed=1
    echo "$@"
}

tidy()
{
    src=$1
    base=$(echo $1 | sed -re 's,[.]sol[.],.pub.,')
    test -f orig.test.cpp && base=orig.test.cpp
    extra=$(perl -ne 'if ( m,// clang-tidy: (.*), ) { print "-checks=$1" }' < $base)

    if ! log $TIDY --header-filter=".*" --warnings-as-errors='*' $extra --quiet $src -- $CXXFLAGS; then
        fail 'tidy failed'; return 1
    else
        return 0
    fi
}

announce()
{
    printf "%-16s " $(echo "$1" | sed -e 's,^sol\.,,;s,.cpp$,,')
}

xsrc()
{
    hdr=$(echo "$@" | xargs -n1 echo | grep -v ^- | grep \.cpp$ | sed -e s,\.cpp,.hpp,)
    for f in $hdr; do if test -f $f; then cat $f; fi; done | \
        perl -ne 'if ( m,// extra files: (.*), ) { print $1 }' | xargs -n1 echo | grep cpp$
}

get_xflags()
{
    XFLAGS="$XFLAGS $(perl -ne 'if ( m,// cxxflags: (.*), ) { print $1 }' < $1)"
}

cxx()
{
    if ! log $CXX $CXXFLAGS "$@" $EXTRA_SRC $(xsrc "$@") $XFLAGS; then
        fail 'build failed'; return 1
    else
        return 0
    fi
}

build()
{
    if test -x $1; then
        ln -f $1 a.out
    else
        if ! cxx "$@"; then return 1; fi
        if ! tidy $1; then return 1; fi
    fi

    return 0
}

grind()
{
    if ! log $WRAPPER $VALGRIND --log-file=valgrind.out --leak-check=full "$@"; then
        test -n "$out" && cat valgrind.out >> $out || cat valgrind.out
        fail 'failed'; return 1;
    fi

    if ! grep -q "ERROR SUMMARY: 0 errors" valgrind.out; then
        test -n "$out" && cat valgrind.out >> $out || cat valgrind.out
        fail "valgrind errors"; return 1;
    fi

    return 0
}

check()
{
    if ! build "$@"; then return 1; fi
    if ! grind ./a.out; then return 1; fi

    echo 'ok'; return 0
}
action=$1 ; shift
source=$1 ; shift
CXXFLAGS="$CXXFLAGS -MD -MP -MF .$(basename $source .cpp).d"

log() { "$@" ; }

$action $source "$@"

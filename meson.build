project(
    'util-graph',
    'ccpp',
    default_options : ['c_std=c++17']
)

cc = meson.get_compiler('cpp')
m_dep = cc.find_library('m', required: false)

deps = [
    dependency('gtk+-3.0'),
    m_dep
]

sources = [
    'util.cpp'
]

executable('util', sources, dependencies: deps)
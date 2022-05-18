import sys
import os
sys.path.insert(0, os.path.abspath('../../'))
sys.path.insert(0, os.path.abspath('./'))

# -- Project information -----------------------------------------------------

project = 'nMPyC'
copyright = '2022, Jonas Schießl and Lisa Krügel'
author = 'Jonas Schießl and Lisa Krügel'

version = '1.0.0'
release = '1.0.0'

# -- General configuration ---------------------------------------------------

# The master toctree document.
master_doc = 'index'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc',
              'sphinx_rtd_theme',
	          'sphinx.ext.intersphinx',
              'sphinx.ext.napoleon',
              'nbsphinx',
              'sphinx.ext.mathjax',
              'sphinx.ext.graphviz',
              'sphinx.ext.autosummary',
              'sphinx.ext.viewcode',
              'sphinx_copybutton',
              'sphinxcontrib.bibtex',
              'recommonmark'
              ]

add_module_names = False

graphviz_output_format = 'svg'

autosummary_generate = True

mathjax3_config = {
    'extensions': ['tex2jax.js'],
    'jax': ['input/TeX', 'output/HTML-CSS'],
}

# Order methods in documentation by their appearence in the sourcecode:
autodoc_member_order = 'bysource'
# Delete this previous line, to order by alphabet.

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', '**.ipynb_checkpoints']


nbsphinx_allow_errors = True
# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'logo_only': True,
    'navigation_depth': 4,
}
html_theme_path = ["../.."]
#html_logo = "static/dompc_var_02_white.svg"
html_show_sourcelink = True

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['static']

# -- Options for LaTeX output ---------------------------------------------
latex_engine ='pdflatex'

latex_documents = [
  ('index', 'nmpyc.tex', u'nMPyC \newline A Python library for solving optimal control problems via MPC',
   u'Jonas Schießl and Lisa Krügel', 'manual'),
]

# -- References -------------------------------------------------
bibtex_bibfiles = ['references.bib']

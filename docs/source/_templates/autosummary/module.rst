{{ objname | escape | underline }}

.. automodule:: {{ fullname }}
.. currentmodule:: {{ fullname }}

{% if classes %}

.. rubric:: Classes
.. autosummary::
    :toctree:
    :nosignatures:
    {% for class in classes %}
    {{ class }}
    {% endfor %}
{% endif %}

{% if functions %}

.. rubric:: Functions
.. autosummary::
    :toctree:
    :nosignatures:
    {% for function in functions %}
    {{ function }}
    {% endfor %}
{% endif %}

{% if modules %}
.. automodule:: {{ fullname }}
.. currentmodule:: {{ fullname }}

.. autosummary::
   :toctree:
   :recursive:
   :template: module.rst
   {% for module in modules %}
   {{ module }}
   {% endfor %}
{% endif %}


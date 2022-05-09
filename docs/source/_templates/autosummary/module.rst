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

{% if attributes %}
.. automodule:: {{ fullname }}
.. currentmodule:: {{ fullname }}

.. autosummary::
   :toctree:
   :recursive:
   :template: attribute.rst
   {% for attribute in attributes %}
   {{ attribute }}
   {% endfor %}
{% endif %}


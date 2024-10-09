FROM python:3.11

RUN pip install --upgrade pip
RUN curl -sSL https://install.python-poetry.org | python -

ENV PATH /root/.local/bin:$PATH
RUN poetry config virtualenvs.in-project true

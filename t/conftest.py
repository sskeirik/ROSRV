import pytest

# https://docs.pytest.org/en/latest/example/simple.html

def pytest_addoption(parser):
    parser.addoption( '--with-rvmaster'
                    , help='Tests Assuming Running RV-Master'
                    , default=False
                    , action='store_true'
                    )
    parser.addoption( '--skip-dl-tests'
                    , help='Skip Dl Based Tests'
                    , default=False
                    , action='store_true'
                    )

def pytest_configure(config):
    config.addinivalue_line("markers", "dlTest: mark as dL Test")

def pytest_collection_modifyitems(config, items):
    if not config.getoption("--skip-dl-tests"):
        return
    skip_dl = pytest.mark.skipif(reason="use --skip-dl-tests option to skip")
    for item in items:
        if "dlTest" in item.keywords:
            item.add_marker(skip_dl)

@pytest.fixture(scope='session')
def get_options(request):
    if request.config.getoption('--with-rvmaster'):
        monitor_topics_list = [ '/monitored', '/chatter', '/color_chatter' ]
        if not request.config.getoption('--skip-dl-tests'):
            monitor_topics_list.extend(['/level_sensor', '/flow_control_cmd'])

        return { 'topic_prefix'        : ''
               , 'monitor_flags'       : '--with-rvmaster'
               , 'monitor_topics_list' : monitor_topics_list }
    else:
        return { 'topic_prefix'  : 'rv/monitored'
               , 'monitor_flags' : ''}


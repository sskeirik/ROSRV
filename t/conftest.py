import pytest

def pytest_addoption(parser):
    parser.addoption( '--with-rvmaster'
                    , help='Tests Assuming Running RV-Master'
                    , default=False
                    , action='store_true'
                    )

@pytest.fixture(scope='session')
def get_options(request):
    if(request.config.getoption('--with-rvmaster')):
        monitor_topics_list = [ '/monitored', '/chatter', '/color_chatter' ]
        return { 'topic_prefix'        : ''
               , 'monitor_flags'       : '--with-rvmaster'
               , 'monitor_topics_list' : monitor_topics_list }
    else:
        return { 'topic_prefix'  : 'rv/monitored'
               , 'monitor_flags' : ''}


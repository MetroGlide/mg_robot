import os

from mg_utils.record_topics import merge_topics, read_topic_file

MONITORING = os.path.join(os.path.dirname(__file__), '..', 'params', 'record_topics_monitoring.txt')


def test_read_topic_file_skips_comments_and_blank_lines(tmp_path):
    path = tmp_path / 'topics.txt'
    path.write_text('# comment\n/odom\n\n  /scan  \n  # indented comment\n/tf\n', encoding='utf-8')
    assert read_topic_file(str(path)) == ['/odom', '/scan', '/tf']


def test_merge_topics_keeps_first_occurrence_order():
    assert merge_topics(['/a', '/b'], ['/b', '/c'], ['/a', '/d']) == ['/a', '/b', '/c', '/d']


def test_monitoring_list_has_the_topics_that_make_log_analysis_possible():
    topics = read_topic_file(MONITORING)
    for required in ('/diagnostics', '/diagnostics_agg', '/rosout', '/events'):
        assert required in topics
    assert len(topics) == len(set(topics))
    assert all(topic.startswith('/') for topic in topics)

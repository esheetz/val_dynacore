# stanza reference: https://stanfordnlp.github.io/stanza/installation_usage.html
import stanza

def initializeParser():
    stanza.download('en', processors='tokenize,pos,lemma,depparse')
    return stanza.Pipeline('en', processors='tokenize,pos,lemma,depparse')

def parse(nlp, command):
    doc = nlp(command)
    dependencies = {}
    for sentence in doc.sentences:
        for word in sentence.words:
            print(word.text, word.pos, word.deprel, word.xpos)
            dependencies[str(word.deprel)] = str(word.text)
    return dependencies

/* This is simple demonstration of how to use expat. This program
   reads an XML document from standard input and writes a line with
   the name of each element to standard output indenting child
   elements by one tab stop more than their parent element.
   It must be used with Expat compiled for UTF-8 output.
                            __  __            _
                         ___\ \/ /_ __   __ _| |_
                        / _ \\  /| '_ \ / _` | __|
                       |  __//  \| |_) | (_| | |_
                        \___/_/\_\ .__/ \__,_|\__|
                                 |_| XML parser

   Copyright (c) 1997-2000 Thai Open Source Software Center Ltd
   Copyright (c) 2001-2003 Fred L. Drake, Jr. <fdrake@users.sourceforge.net>
   Copyright (c) 2004-2006 Karl Waclawek <karl@waclawek.net>
   Copyright (c) 2005-2007 Steven Solie <steven@solie.ca>
   Copyright (c) 2016-2022 Sebastian Pipping <sebastian@pipping.org>
   Copyright (c) 2017      Rhodri James <rhodri@wildebeest.org.uk>
   Copyright (c) 2019      Zhongyuan Zhou <zhouzhongyuan@huawei.com>
   Licensed under the MIT license:

   Permission is  hereby granted,  free of charge,  to any  person obtaining
   a  copy  of  this  software   and  associated  documentation  files  (the
   "Software"),  to  deal in  the  Software  without restriction,  including
   without  limitation the  rights  to use,  copy,  modify, merge,  publish,
   distribute, sublicense, and/or sell copies of the Software, and to permit
   persons  to whom  the Software  is  furnished to  do so,  subject to  the
   following conditions:

   The above copyright  notice and this permission notice  shall be included
   in all copies or substantial portions of the Software.

   THE  SOFTWARE  IS  PROVIDED  "AS  IS",  WITHOUT  WARRANTY  OF  ANY  KIND,
   EXPRESS  OR IMPLIED,  INCLUDING  BUT  NOT LIMITED  TO  THE WARRANTIES  OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
   NO EVENT SHALL THE AUTHORS OR  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR  OTHER LIABILITY, WHETHER  IN AN  ACTION OF CONTRACT,  TORT OR
   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
   USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <expat.h>

#ifdef XML_LARGE_SIZE
#  define XML_FMT_INT_MOD "ll"
#else
#  define XML_FMT_INT_MOD "l"
#endif

#ifdef XML_UNICODE_WCHAR_T
#  define XML_FMT_STR "ls"
#else
#  define XML_FMT_STR "s"
#endif


# define NODE_STR "node"
# define WAY_STR "way"
# define RELATION_STR "relation"
# define TAG_STR "tag"
# define MEMBER_STR "member"

struct parser_data {
  size_t depth;
  size_t nodes;
  size_t ways;
  size_t relations;
  size_t tags;
  size_t members;
  size_t others;
};


static void XMLCALL
startElement(void *userData, const XML_Char *name, const XML_Char **atts) {
  int i;
  struct parser_data* parserData = (struct parser_data *)userData;
  (void)atts;


  if (strcmp(name, NODE_STR) == 0) {  
    parserData->nodes += 1;
    putchar('n');
  } else if (strcmp(name, WAY_STR) == 0) {
    putchar('w');
    parserData->ways += 1;
    } else if (strcmp(name, RELATION_STR) == 0) {
    putchar('r');
    parserData->relations += 1;
    } else if (strcmp(name, TAG_STR) == 0) {
    putchar('t');
    parserData->tags += 1;
    } else if (strcmp(name, MEMBER_STR) == 0) {
    parserData->members += 1;
    putchar('m');
  } else {
    parserData->others += 1;
    putchar('o');
  }


  for (i = 0; i < parserData->depth; i++) {
    putchar('|');
    putchar('\t');
    
  }
  printf("%" XML_FMT_STR "\n",  name);
  parserData->depth += 1;
}

static void XMLCALL
endElement(void *userData, const XML_Char *name) {
  struct parser_data* parserData = (struct parser_data *)userData;
  (void)name;

  parserData->depth -= 1;
}

int
main(int argc, char** argv) {
  XML_Parser parser = XML_ParserCreate(NULL);
  int done;
  int depth = 0;
  struct parser_data userData  = {0, 0};

  if (! parser) {
    fprintf(stderr, "Couldn't allocate memory for parser\n");
    return 1;
  }

  if (argc < 2) {
    fprintf(stderr, "Usage: %s filename\n", argv[0]);
    return 1;
  }

  FILE* file = fopen(argv[1], "r");

  XML_SetUserData(parser, &userData);
  XML_SetElementHandler(parser, startElement, endElement);

  do {
    void *const buf = XML_GetBuffer(parser, BUFSIZ);
    if (! buf) {
      fprintf(stderr, "Couldn't allocate memory for buffer\n");
      XML_ParserFree(parser);
      return 1;
    }

    const size_t len = fread(buf, 1, BUFSIZ, file);

    if (ferror(file)) {
      fprintf(stderr, "Read error\n");
      XML_ParserFree(parser);
      return 1;
    }

    done = feof(file);

    if (XML_ParseBuffer(parser, (int)len, done) == XML_STATUS_ERROR) {
      fprintf(stderr,
              "Parse error at line %" XML_FMT_INT_MOD "u:\n%" XML_FMT_STR "\n",
              XML_GetCurrentLineNumber(parser),
              XML_ErrorString(XML_GetErrorCode(parser)));
      XML_ParserFree(parser);
      return 1;
    }
  } while (! done);
    printf("Nodes: %zu\n", userData.nodes);
    printf("Ways: %zu\n", userData.ways);
    printf("Relations: %zu\n", userData.relations);
    printf("Tags: %zu\n", userData.tags);
    printf("Members: %zu\n", userData.members);
    printf("Others: %zu\n", userData.others);
    printf("Total: %zu\n", userData.nodes + userData.ways + userData.relations + userData.tags + userData.members + userData.others);
    
    
  XML_ParserFree(parser);
  return 0;
}
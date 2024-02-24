#include <sqlite3.h>
#include <stdio.h>

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  // open DB
  sqlite3 * db = NULL;
  if (sqlite3_open("data.db", &db) != SQLITE_OK) {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // create Table
  if (sqlite3_exec(
      db, "CREATE TABLE IF NOT EXISTS CountList (name PRIMARY KEY, count);", NULL,
      NULL, NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // insert record
  if (sqlite3_exec(
      db, "INSERT OR IGNORE INTO CountList VALUES ('boot', 0);", NULL, NULL,
      NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // update record
  if (sqlite3_exec(
      db, "UPDATE CountList SET count = count + 1 WHERE name == 'boot'", NULL, NULL,
      NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  printf("Done\n");
  return 0;
}

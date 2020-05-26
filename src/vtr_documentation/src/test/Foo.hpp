/** \brief ASRL namespace brief description.
 *
 * ASRL namespace detailed description.
 */
namespace asrl {

/** \brief Documentation namespace brief description.
 *
 * This is the namespace containing the documentation.
 */
namespace documentation {

/** \brief foo class brief description.
 *
 * Detailed foo description with a reference to \ref Foo_tutorial
 */
class Foo {
 public:
  /** \brief Default constructor.
   *
   * The default constructor throws a table (╯°o°）╯︵ ┻━┻.
   */
  Foo();

 protected:
  /** \brief bar function brief description
   *
   * bar does this. Bar resets the table ┳━┳ ノ( °-°ノ).
   */
  int bar(const int& var1,  ///< input arg 1
          const int& var2,  ///< input arg 2
          int& var3         ///< output arg 1
  );
};

}  // namespace documentation
}  // namespace asrl
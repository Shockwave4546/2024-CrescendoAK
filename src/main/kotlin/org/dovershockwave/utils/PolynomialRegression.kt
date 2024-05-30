/*
 * Copyright (c) 2023 FRC Team 4546
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.dovershockwave.utils

import Jama.Matrix
import Jama.QRDecomposition
import kotlin.math.abs
import kotlin.math.pow

// NOTE: This file is available at http://algs4.cs.princeton.edu/14analysis/PolynomialRegression.java.html (Rewritten in Kotlin)

/**
 * The {@code PolynomialRegression} class performs a polynomial regression on an set of <em>N</em>
 * data points (<em>y<sub>i</sub></em>, <em>x<sub>i</sub></em>). That is, it fits a polynomial
 * <em>y</em> = &beta;<sub>0</sub> + &beta;<sub>1</sub> <em>x</em> + &beta;<sub>2</sub>
 * <em>x</em><sup>2</sup> + ... + &beta;<sub><em>d</em></sub> <em>x</em><sup><em>d</em></sup> (where
 * <em>y</em> is the response variable, <em>x</em> is the predictor variable, and the
 * &beta;<sub><em>i</em></sub> are the regression coefficients) that minimizes the sum of squared
 * residuals of the multiple regression model. It also computes associated the coefficient of
 * determination <em>R</em><sup>2</sup>.
 *
 * <p>This implementation performs a QR-decomposition of the underlying Vandermonde matrix, so it is
 * neither the fastest nor the most numerically stable way to perform the polynomial regression.
 *
 * @author Robert Sedgewick
 * @author Kevin Wayne
 */
class PolynomialRegression(x: DoubleArray, y: DoubleArray, private var degree: Int, private val variableName: String) : Comparable<PolynomialRegression> {
  private var beta: Matrix // the polynomial regression coefficients
  private var sse: Double // sum of squares due to error
  private var sst = 0.0 // total sum of squares

  init {
    val n = x.size
    var qr: QRDecomposition
    var matrixX: Matrix

    // in case Vandermonde matrix does not have full rank, reduce degree until it does
    while (true) {
      // build Vandermonde matrix
      val vandermonde = Array(n) { DoubleArray(this.degree + 1) }
      for (i in 0 until n) {
        for (j in 0..this.degree) {
          vandermonde[i][j] = x[i].pow(j.toDouble())
        }
      }

      matrixX = Matrix(vandermonde)

      // find least squares solution
      qr = QRDecomposition(matrixX)
      if (qr.isFullRank()) break

      // decrease degree and try again
      this.degree--
    }

    // create matrix from vector
    val matrixY = Matrix(y, n)

    // linear regression coefficients
    beta = qr.solve(matrixY)

    // mean of y[] values
    var sum = 0.0
    for (i in 0 until n) sum += y[i]
    val mean = sum / n

    // total variation to be accounted for
    for (i in 0 until n) {
      val dev = y[i] - mean
      sst += dev * dev
    }

    // variation not accounted for
    val residuals = matrixX.times(beta).minus(matrixY)
    sse = residuals.norm2() * residuals.norm2()
  }

  /**
   * Returns the {@code j}th regression coefficient.
   *
   * @param j the index
   * @return the {@code j}th regression coefficient
   */
  fun beta(j: Int): Double {
    // to make -0.0 print as 0.0
    if (abs(beta.get(j, 0)) < 1E-4) return 0.0
    return beta.get(j, 0)
  }

  /**
   * Returns the degree of the polynomial to fit.
   *
   * @return the degree of the polynomial to fit
   */
  fun degree(): Int {
    return degree
  }

  /**
   * Returns the coefficient of determination <em>R</em><sup>2</sup>.
   *
   * @return the coefficient of determination <em>R</em><sup>2</sup>, which is a real number between
   *     0 and 1
   */
  fun r2(): Double {
    if (sst == 0.0) return 1.0 // constant function
    return 1.0 - sse / sst
  }

  /**
   * Returns the expected response {@code y} given the value of the predictor variable {@code x}.
   *
   * @param x the value of the predictor variable
   * @return the expected response {@code y} given the value of the predictor variable {@code x}
   */
  fun predict(x: Double): Double {
    // Horner's method
    var y = 0.0
    for (j in degree downTo 0) y = beta(j) + x * y
    return y
  }

  /**
   * Returns a string representation of the polynomial regression model.
   *
   * @return a string representation of the polynomial regression model, including the best-fit
   *     polynomial and the coefficient of determination <em>R</em><sup>2</sup>
   */
  override fun toString() = buildString {
    var j = degree

    // ignoring leading zero coefficients
    while (j >= 0 && abs(beta(j)) < 1E-5) j--

    // create remaining terms
    while (j >= 0) {
      when (j) {
        0 -> append(String.format("%.10f ", beta(j)))
        1 -> append(String.format("%.10f %s + ", beta(j), variableName))
        else -> append(String.format("%.10f %s^%d + ", beta(j), variableName, j))
      }
      j--
    }

    append("  (R^2 = " + String.format("%.3f", r2()) + ")")
  }.replace("+ -", "- ")

  /** Compare lexicographically. */
  override fun compareTo(other: PolynomialRegression): Int {
    val epsilon = 1E-5
    val maxDegree = this.degree().coerceAtLeast(other.degree())
    for (j in maxDegree downTo 0) {
      var term1 = 0.0
      var term2 = 0.0
      if (this.degree() >= j) term1 = this.beta(j)
      if (other.degree() >= j) term2 = other.beta(j)
      if (abs(term1) < epsilon) term1 = 0.0
      if (abs(term2) < epsilon) term2 = 0.0
      if (term1 < term2) return -1
      else if (term1 > term2) return 1
    }

    return 0
  }
}

fun main() {
  val x = doubleArrayOf(10.0, 20.0, 40.0, 80.0, 160.0, 200.0)
  val y = doubleArrayOf(100.0, 350.0, 1500.0, 6700.0, 20160.0, 40000.0)
  val regression = PolynomialRegression(x, y, 3, "n")

  // 0.0092130795 n^3 - 1.6395342395 n^2 + 168.9231547456 n - 2113.7305678304   (R^2 = 0.997)
  println(regression)
}